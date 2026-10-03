//! Convert an SVG's `<path>` outlines into `movepath` waypoint files.
//!
//! Dependency-free single file, built with plain `rustc` (it is a dev-time
//! conversion tool, not part of the workspace):
//!
//! ```text
//! rustc -O scripts/utilities/svg_to_waypoints.rs -o /tmp/svg_to_waypoints
//! /tmp/svg_to_waypoints art.svg --out-dir contours/art \
//!     --fit 300 --tolerance 0.5 --min-spacing 2.0
//! scripts/utilities/draw_svg.sh contours/art
//! ```
//!
//! Run both from the workspace root: `session.txt` refers to the contour files
//! by the `--out-dir` path as given.
//!
//! ## Why one file per subpath
//!
//! `movepath` takes a *single* continuous spline through every waypoint —
//! there is no pen-up/pen-down. An SVG outline is a set of independent
//! closed contours (letters, and the holes inside them), so collapsing them
//! into one file would draw straight lines between unrelated contours. Each
//! subpath therefore becomes its own file, plus a `session.txt` that issues
//! them in order.
//!
//! ## Geometry handled
//!
//! - `M/m L/l H/h V/v C/c S/s Z/z`. Quadratics and arcs are *not* handled;
//!   the tool errors out rather than approximating silently. (potrace output
//!   only emits the above.)
//! - The single `transform="translate(tx,ty) scale(sx,sy)"` form that
//!   potrace emits on its wrapping `<g>`. Anything else is rejected — a
//!   silently-ignored transform would flip or shrink the artwork without
//!   saying so.
//! - Cubics are flattened by recursive subdivision to a chord tolerance,
//!   then thinned to a minimum spacing, both in *output* (post-scaling)
//!   units so the numbers mean millimetres.

use std::fmt::Write as _;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(msg) => println!("{msg}"),
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(1);
        }
    }
}

struct Options {
    input: String,
    out_dir: String,
    /// Longest side of the artwork's bounding box, in output units (mm).
    fit: f64,
    /// Max chord deviation when flattening a cubic, in output units.
    tolerance: f64,
    /// Waypoints closer together than this are dropped, in output units.
    min_spacing: f64,
    /// Centre of the artwork in output units, i.e. where the drawing lands.
    center: (f64, f64),
    /// Drop contours whose bounding box is smaller than this — traced
    /// bitmaps carry specks that are noise, not geometry.
    min_extent: f64,
    /// Negate y on output. **Default on**, and it has to be: applying the
    /// `<g>` transform lands you in SVG *user* space, whose y axis points
    /// down, while the machine frame's points up. Without this the artwork
    /// comes out mirrored top-to-bottom. `--no-flip-y` opts out.
    flip_y: bool,
}

fn run(args: &[String]) -> Result<String, String> {
    let opts = parse_args(args)?;
    let svg = std::fs::read_to_string(&opts.input)
        .map_err(|e| format!("failed to read {:?}: {e}", opts.input))?;

    let transform = parse_group_transform(&svg)?;
    let mut contours: Vec<Vec<(f64, f64)>> = Vec::new();
    for d in extract_path_data(&svg) {
        for sub in flatten_path(&d, transform)? {
            contours.push(sub);
        }
    }
    if contours.is_empty() {
        return Err("no <path d=...> subpaths found".into());
    }

    // Scale/centre every contour together, so relative placement survives.
    let (min, max) = bounds(&contours);
    let span = (max.0 - min.0).max(max.1 - min.1);
    if span <= 0.0 {
        return Err("artwork has zero extent".into());
    }
    let s = opts.fit / span;
    let mid = ((min.0 + max.0) / 2.0, (min.1 + max.1) / 2.0);
    let sy = if opts.flip_y { -s } else { s };
    for c in &mut contours {
        for p in c.iter_mut() {
            p.0 = quantize((p.0 - mid.0) * s + opts.center.0);
            p.1 = quantize((p.1 - mid.1) * sy + opts.center.1);
        }
    }

    // Flattening ran in source units, so re-thin now that we know the scale.
    let mut kept: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut dropped = 0usize;
    for c in contours {
        let (cmin, cmax) = bounds(std::slice::from_ref(&c));
        if (cmax.0 - cmin.0).max(cmax.1 - cmin.1) < opts.min_extent {
            dropped += 1;
            continue;
        }
        let thinned = thin(&c, opts.min_spacing);
        // 3, not 2: the first point becomes the rapid's target rather than
        // a line in the file (see below), and PathProfile still needs 2
        // distinct waypoints after that.
        if thinned.len() >= 3 {
            kept.push(thinned);
        } else {
            dropped += 1;
        }
    }
    if kept.is_empty() {
        return Err("every contour was filtered out; loosen --min-extent".into());
    }

    std::fs::create_dir_all(&opts.out_dir)
        .map_err(|e| format!("failed to create {:?}: {e}", opts.out_dir))?;

    let mut session = String::new();
    let mut total = 0usize;
    // Total distance the tool will travel — contour arc lengths plus the
    // rapids between them. Written to `length.txt` so a driver script can
    // wait the right amount of time: `app` quits on stdin EOF, so a
    // too-short wait silently truncates the drawing.
    let mut travel = 0.0f64;
    let mut pen = (0.0f64, 0.0f64);
    for (i, c) in kept.iter().enumerate() {
        let name = format!("contour{i:02}.txt");
        // The file starts at the *second* point. `install_path_move`
        // prepends the group's current commanded position as waypoint 0,
        // and the rapid below has just parked it on the first point — so
        // including it too makes waypoints 0 and 1 identical, which is a
        // zero-length segment and a hard rejection at promotion time.
        let mut body = String::new();
        for p in &c[1..] {
            writeln!(body, "{:.3} {:.3}", p.0, p.1).unwrap();
        }
        let file = format!("{}/{name}", opts.out_dir);
        std::fs::write(&file, body).map_err(|e| format!("failed to write {file:?}: {e}"))?;
        total += c.len();

        // Rapid to the contour start with a straight-line group `move`,
        // then trace it. Both buffered so the whole set runs back-to-back
        // from one piped session. (A `movepath` can't do the rapid: its
        // first token after the group is a *waypoint count*.)
        writeln!(
            session,
            "move axisGroup0 {:.3} {:.3} buffered",
            c[0].0, c[0].1
        )
        .unwrap();
        writeln!(session, "movepath axisGroup0 file {file} buffered").unwrap();

        travel += hypot(pen, c[0]);
        for w in c.windows(2) {
            travel += hypot(w[0], w[1]);
        }
        pen = *c.last().unwrap();
    }
    let session_path = format!("{}/session.txt", opts.out_dir);
    std::fs::write(&session_path, session)
        .map_err(|e| format!("failed to write {session_path:?}: {e}"))?;
    let length_path = format!("{}/length.txt", opts.out_dir);
    std::fs::write(&length_path, format!("{travel:.0}\n"))
        .map_err(|e| format!("failed to write {length_path:?}: {e}"))?;

    Ok(format!(
        "{} contours, {total} waypoints, {travel:.0} mm of travel -> {} \
         (dropped {dropped} tiny contour(s))",
        kept.len(),
        opts.out_dir
    ))
}

fn parse_args(args: &[String]) -> Result<Options, String> {
    let mut opts = Options {
        input: String::new(),
        out_dir: String::new(),
        fit: 300.0,
        tolerance: 0.5,
        min_spacing: 2.0,
        center: (0.0, 0.0),
        min_extent: 5.0,
        flip_y: true,
    };
    let mut i = 0;
    while i < args.len() {
        let a = args[i].as_str();
        let mut next = |name: &str| -> Result<String, String> {
            i += 1;
            args.get(i)
                .cloned()
                .ok_or_else(|| format!("{name} needs a value"))
        };
        match a {
            "--out-dir" => opts.out_dir = next("--out-dir")?,
            "--fit" => opts.fit = num(&next("--fit")?)?,
            "--tolerance" => opts.tolerance = num(&next("--tolerance")?)?,
            "--min-spacing" => opts.min_spacing = num(&next("--min-spacing")?)?,
            "--min-extent" => opts.min_extent = num(&next("--min-extent")?)?,
            "--no-flip-y" => opts.flip_y = false,
            "--center" => {
                let x = num(&next("--center")?)?;
                let y = num(&next("--center")?)?;
                opts.center = (x, y);
            }
            _ if a.starts_with("--") => return Err(format!("unknown option {a}")),
            _ if opts.input.is_empty() => opts.input = a.to_string(),
            _ => return Err(format!("unexpected argument {a}")),
        }
        i += 1;
    }
    if opts.input.is_empty() {
        return Err("usage: svg_to_waypoints <file.svg> --out-dir <dir> [--fit mm] \
                    [--tolerance mm] [--min-spacing mm] [--min-extent mm] [--center x y] \
                    [--no-flip-y]"
            .into());
    }
    if opts.out_dir.is_empty() {
        opts.out_dir = "waypoints".into();
    }
    Ok(opts)
}

fn num(s: &str) -> Result<f64, String> {
    s.parse::<f64>().map_err(|e| format!("{s:?}: {e}"))
}

/// `(tx, ty, sx, sy)` from the wrapping `<g transform=...>`, identity if
/// there is no transform at all.
fn parse_group_transform(svg: &str) -> Result<(f64, f64, f64, f64), String> {
    let Some(rest) = svg.split("transform=\"").nth(1) else {
        return Ok((0.0, 0.0, 1.0, 1.0));
    };
    let spec = rest.split('"').next().unwrap_or("").trim().to_string();
    let mut t = (0.0, 0.0, 1.0, 1.0);
    let mut seen = String::new();
    let mut tail = spec.as_str();
    while let Some(open) = tail.find('(') {
        let name = tail[..open].trim().trim_start_matches(|c: char| !c.is_alphabetic());
        let close = tail[open..]
            .find(')')
            .ok_or_else(|| format!("unbalanced transform {spec:?}"))?
            + open;
        let nums: Vec<f64> = tail[open + 1..close]
            .split(|c: char| c == ',' || c.is_whitespace())
            .filter(|s| !s.is_empty())
            .map(num)
            .collect::<Result<_, _>>()?;
        match (name, nums.len()) {
            ("translate", 1) => t.0 = nums[0],
            ("translate", 2) => {
                t.0 = nums[0];
                t.1 = nums[1];
            }
            ("scale", 1) => {
                t.2 = nums[0];
                t.3 = nums[0];
            }
            ("scale", 2) => {
                t.2 = nums[0];
                t.3 = nums[1];
            }
            _ => {
                return Err(format!(
                    "unsupported transform {name:?} with {} argument(s) in {spec:?}; \
                     only translate/scale are handled",
                    nums.len()
                ))
            }
        }
        seen.push_str(name);
        tail = &tail[close + 1..];
    }
    if seen.is_empty() {
        return Err(format!("could not parse transform {spec:?}"));
    }
    Ok(t)
}

fn extract_path_data(svg: &str) -> Vec<String> {
    let mut out = Vec::new();
    for chunk in svg.split("<path").skip(1) {
        if let Some(rest) = chunk.split("d=\"").nth(1) {
            if let Some(d) = rest.split('"').next() {
                out.push(d.to_string());
            }
        }
    }
    out
}

/// Tokenize path data into commands and their numbers.
fn tokenize(d: &str) -> Result<Vec<(char, Vec<f64>)>, String> {
    let mut cmds: Vec<(char, Vec<f64>)> = Vec::new();
    let bytes: Vec<char> = d.chars().collect();
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i];
        if c.is_whitespace() || c == ',' {
            i += 1;
        } else if c.is_ascii_alphabetic() {
            cmds.push((c, Vec::new()));
            i += 1;
        } else {
            let start = i;
            if bytes[i] == '+' || bytes[i] == '-' {
                i += 1;
            }
            while i < bytes.len() && (bytes[i].is_ascii_digit() || bytes[i] == '.') {
                i += 1;
            }
            if i < bytes.len() && (bytes[i] == 'e' || bytes[i] == 'E') {
                i += 1;
                if i < bytes.len() && (bytes[i] == '+' || bytes[i] == '-') {
                    i += 1;
                }
                while i < bytes.len() && bytes[i].is_ascii_digit() {
                    i += 1;
                }
            }
            if i == start {
                return Err(format!("unparsable character {:?} in path data", bytes[i]));
            }
            let text: String = bytes[start..i].iter().collect();
            let v = num(&text)?;
            cmds.last_mut()
                .ok_or_else(|| format!("number {text:?} before any command"))?
                .1
                .push(v);
        }
    }
    Ok(cmds)
}

/// Flatten one path's data into polylines, one per subpath, already
/// transformed by the `<g>` transform. Flattening tolerance is applied in
/// *source* units here; `run` re-thins after scaling.
fn flatten_path(d: &str, t: (f64, f64, f64, f64)) -> Result<Vec<Vec<(f64, f64)>>, String> {
    let xf = |p: (f64, f64)| (p.0 * t.2 + t.0, p.1 * t.3 + t.1);
    let mut subpaths: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut cur: Vec<(f64, f64)> = Vec::new();
    // Current point and subpath start, in *source* (pre-transform) units.
    let mut pt = (0.0, 0.0);
    let mut start = (0.0, 0.0);
    // Previous cubic's second control point, for `S`/`s`.
    let mut prev_ctrl: Option<(f64, f64)> = None;

    for (cmd, nums) in tokenize(d)? {
        let rel = cmd.is_lowercase();
        let up = cmd.to_ascii_uppercase();
        let arity = match up {
            'M' | 'L' => 2,
            'H' | 'V' => 1,
            'C' => 6,
            'S' => 4,
            'Z' => 0,
            'Q' | 'T' | 'A' => {
                return Err(format!(
                    "path command {cmd:?} (quadratic/arc) is not supported"
                ))
            }
            _ => return Err(format!("unknown path command {cmd:?}")),
        };
        if arity == 0 {
            if !cur.is_empty() {
                cur.push(xf(start));
                subpaths.push(std::mem::take(&mut cur));
            }
            pt = start;
            prev_ctrl = None;
            continue;
        }
        if nums.is_empty() || nums.len() % arity != 0 {
            return Err(format!(
                "command {cmd:?} expects a multiple of {arity} numbers, got {}",
                nums.len()
            ));
        }
        for (k, group) in nums.chunks(arity).enumerate() {
            let abs = |i: usize, base: f64| if rel { base + group[i] } else { group[i] };
            match up {
                // A repeated M coordinate pair means implicit L (SVG spec).
                'M' if k == 0 => {
                    if !cur.is_empty() {
                        subpaths.push(std::mem::take(&mut cur));
                    }
                    pt = (abs(0, pt.0), abs(1, pt.1));
                    start = pt;
                    cur.push(xf(pt));
                    prev_ctrl = None;
                }
                'M' | 'L' => {
                    pt = (abs(0, pt.0), abs(1, pt.1));
                    cur.push(xf(pt));
                    prev_ctrl = None;
                }
                'H' => {
                    pt = (abs(0, pt.0), pt.1);
                    cur.push(xf(pt));
                    prev_ctrl = None;
                }
                'V' => {
                    pt = (pt.0, abs(0, pt.1));
                    cur.push(xf(pt));
                    prev_ctrl = None;
                }
                'C' | 'S' => {
                    let (c1, c2, end) = if up == 'C' {
                        (
                            (abs(0, pt.0), abs(1, pt.1)),
                            (abs(2, pt.0), abs(3, pt.1)),
                            (abs(4, pt.0), abs(5, pt.1)),
                        )
                    } else {
                        // S: first control point mirrors the previous one.
                        let c1 = match prev_ctrl {
                            Some(p) => (2.0 * pt.0 - p.0, 2.0 * pt.1 - p.1),
                            None => pt,
                        };
                        (
                            c1,
                            (abs(0, pt.0), abs(1, pt.1)),
                            (abs(2, pt.0), abs(3, pt.1)),
                        )
                    };
                    if cur.is_empty() {
                        cur.push(xf(pt));
                    }
                    flatten_cubic(xf(pt), xf(c1), xf(c2), xf(end), &mut cur, 0);
                    prev_ctrl = Some(c2);
                    pt = end;
                }
                _ => unreachable!(),
            }
        }
    }
    if !cur.is_empty() {
        subpaths.push(cur);
    }
    Ok(subpaths)
}

/// Source-unit flattening tolerance. Deliberately tight — `run` thins the
/// result in output units, where the number is meaningful, so being
/// generous here would just lose detail early.
const FLATTEN_TOL: f64 = 0.05;
const MAX_DEPTH: u32 = 16;

/// Recursive de Casteljau subdivision until the control points sit within
/// `FLATTEN_TOL` of the chord. Appends the end point, never the start (the
/// caller already has it).
fn flatten_cubic(
    p0: (f64, f64),
    p1: (f64, f64),
    p2: (f64, f64),
    p3: (f64, f64),
    out: &mut Vec<(f64, f64)>,
    depth: u32,
) {
    let flat = depth >= MAX_DEPTH
        || (dist_to_line(p1, p0, p3).max(dist_to_line(p2, p0, p3)) <= FLATTEN_TOL);
    if flat {
        out.push(p3);
        return;
    }
    let mid = |a: (f64, f64), b: (f64, f64)| ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
    let p01 = mid(p0, p1);
    let p12 = mid(p1, p2);
    let p23 = mid(p2, p3);
    let p012 = mid(p01, p12);
    let p123 = mid(p12, p23);
    let m = mid(p012, p123);
    flatten_cubic(p0, p01, p012, m, out, depth + 1);
    flatten_cubic(m, p123, p23, p3, out, depth + 1);
}

fn dist_to_line(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len = (dx * dx + dy * dy).sqrt();
    if len < f64::EPSILON {
        let (ex, ey) = (p.0 - a.0, p.1 - a.1);
        return (ex * ex + ey * ey).sqrt();
    }
    ((p.0 - a.0) * dy - (p.1 - a.1) * dx).abs() / len
}

/// Drop points closer than `min_spacing` to the last kept one.
///
/// The contour's final point is preserved exactly — it closes the loop — but
/// by *replacing* the previous kept point when it lands too close, not by
/// appending. Appending would leave a sub-millimetre last segment whose
/// curvature dwarfs the rest of the contour.
fn thin(points: &[(f64, f64)], min_spacing: f64) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = Vec::new();
    for (i, &p) in points.iter().enumerate() {
        let last = out.last().copied();
        let far = match last {
            None => true,
            Some(q) => hypot(p, q) >= min_spacing,
        };
        if far {
            out.push(p);
        } else if i + 1 == points.len() && out.len() > 1 {
            // Too close to keep as its own waypoint, but it is the
            // endpoint: move the last kept waypoint onto it.
            *out.last_mut().unwrap() = p;
        }
    }
    // Belt and braces: `quantize` should already have made an exact
    // duplicate impossible, but a zero-length segment is a hard rejection
    // downstream, so never let one reach the file.
    out.dedup_by(|a, b| a.0 == b.0 && a.1 == b.1);
    out
}

/// Snap to the precision the waypoint files are *written* at.
///
/// Not cosmetic: `thin`'s duplicate check has to see the same numbers
/// `PathProfile` will. Two points 0.0004 mm apart survive a full-precision
/// dedupe, then print identically at `{:.3}` — and a zero-length segment
/// divides by zero in the centripetal knot spacing, so the path move is
/// rejected at promotion time. Quantizing here means the check and the file
/// agree.
const OUTPUT_DECIMALS: f64 = 1000.0;

fn quantize(v: f64) -> f64 {
    (v * OUTPUT_DECIMALS).round() / OUTPUT_DECIMALS
}

fn hypot(a: (f64, f64), b: (f64, f64)) -> f64 {
    ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt()
}

fn bounds(contours: &[Vec<(f64, f64)>]) -> ((f64, f64), (f64, f64)) {
    let mut min = (f64::INFINITY, f64::INFINITY);
    let mut max = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for c in contours {
        for p in c {
            min.0 = min.0.min(p.0);
            min.1 = min.1.min(p.1);
            max.0 = max.0.max(p.0);
            max.1 = max.1.max(p.1);
        }
    }
    (min, max)
}
