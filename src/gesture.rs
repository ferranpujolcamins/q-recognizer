/**
 * The $P Point-Cloud Recognizer (rust version)
 * 
 * Translated to rust from the original authors' C# code with an AI tool.
 * The translated code has been reviewed by Ferran Pujol Camins.
 *
 * Original authors:
 * 
 * 	    Radu-Daniel Vatavu, Ph.D.
 *	    University Stefan cel Mare of Suceava
 *	    Suceava 720229, Romania
 *	    vatavu@eed.usv.ro
 *
 *	    Lisa Anthony, Ph.D.
 *      UMBC
 *      Information Systems Department
 *      1000 Hilltop Circle
 *      Baltimore, MD 21250
 *      lanthony@umbc.edu
 *
 *	    Jacob O. Wobbrock, Ph.D.
 * 	    The Information School
 *	    University of Washington
 *	    Seattle, WA 98195-2840
 *	    wobbrock@uw.edu
 *
 * The academic publication for the $P recognizer, and what should be 
 * used to cite it, is:
 *
 *	Vatavu, R.-D., Anthony, L. and Wobbrock, J.O. (2012).  
 *	  Gestures as point clouds: A $P recognizer for user interface 
 *	  prototypes. Proceedings of the ACM Int'l Conference on  
 *	  Multimodal Interfaces (ICMI '12). Santa Monica, California  
 *	  (October 22-26, 2012). New York: ACM Press, pp. 273-280.
 *
 * This software is distributed under the "New BSD License" agreement:
 *
 * Copyright (c) 2012, Radu-Daniel Vatavu, Lisa Anthony, and 
 * Jacob O. Wobbrock. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the names of the University Stefan cel Mare of Suceava, 
 *	    University of Washington, nor UMBC, nor the names of its contributors 
 *	    may be used to endorse or promote products derived from this software 
 *	    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Radu-Daniel Vatavu OR Lisa Anthony
 * OR Jacob O. Wobbrock OR Ferran Pujol Camins BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
**/

use crate::{geometry, point::Point};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Default number of points on the gesture path
const SAMPLING_RESOLUTION: usize = 64;
/// Each point has two additional x and y integer coordinates in the interval [0..MAX_INT_COORDINATES-1] used to operate the LUT table efficiently (O(1))
const MAX_INT_COORDINATES: usize = 1024;
/// The default size of the lookup table is 64 x 64
pub const LUT_SIZE: usize = 64;
/// Scale factor to convert between integer x and y coordinates and the size of the LUT
pub const LUT_SCALE_FACTOR: usize = MAX_INT_COORDINATES / LUT_SIZE;

/// Implements a gesture as a cloud of points (i.e., an unordered set of points).
/// For $P, gestures are normalized with respect to scale, translated to origin, and resampled into a fixed number of 32 points.
/// For $Q, a LUT is also computed.
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Gesture {
    /// Gesture class
    pub name: String,
    /// Gesture points (normalized)
    pub points: Vec<Point>,
    /// Look-up table
    #[cfg_attr(feature = "serde", serde(skip))]
    pub lut: Option<Vec<Vec<usize>>>,
}

impl Gesture {
    /// Constructs a new gesture from a list of points and a name
    pub fn new(pts: Vec<Point>, name: &str) -> Self {
        let mut g = Self {
            points: pts,
            name: name.into(),
            lut: None,
        };
        g.normalize();
        g
    }

    /// Normalizes the gesture path. 
    /// The $Q recognizer requires an extra normalization step, the computation of the LUT, 
    /// which can be enabled with the computeLUT parameter.
    pub fn normalize(&mut self) {
        // standard $-family processing: resample, scale, and translate to origin
        self.points = Self::resample(&self.points, SAMPLING_RESOLUTION);
        self.points = Self::scale(&self.points);
        let c = Self::centroid(&self.points);
        self.points = Self::translate_to(&self.points, &c);

        // constructs a lookup table for fast lower bounding (used by $Q)
        self.transform_coordinates_to_integers();
        self.construct_lut();
    }

    /// Performs scale normalization with shape preservation into [0..1]x[0..1]
    fn scale(points: &[Point]) -> Vec<Point> {
        let (mut minx, mut miny) = (f32::MAX, f32::MAX);
        let (mut maxx, mut maxy) = (f32::MIN, f32::MIN);
        for p in points {
            if p.x < minx { minx = p.x; }
            if p.y < miny { miny = p.y; }
            if p.x > maxx { maxx = p.x; }
            if p.y > maxy { maxy = p.y; }
        }
        let scale = (maxx - minx).max(maxy - miny);
        points.iter().map(|p| {
            Point::new((p.x - minx) / scale, (p.y - miny) / scale, p.stroke_id)
        }).collect()
    }

    /// Translates the array of points by p
    fn translate_to(points: &[Point], p: &Point) -> Vec<Point> {
        points.iter().map(|point| {
            Point::new(point.x - p.x, point.y - p.y, p.stroke_id)
        }).collect()
    }

    /// Computes the centroid for an array of points
    fn centroid(points: &[Point]) -> Point {
        let mut cx = 0.0;
        let mut cy = 0.0;
        for p in points {
            cx += p.x;
            cy += p.y;
        }
        let n = points.len() as f32;
        Point::new(cx / n, cy / n, 0)
    }

    /// Resamples the array of points into n equally-distanced points
    fn resample(points: &[Point], n: usize) -> Vec<Point> {
        let mut new_points = Vec::with_capacity(n);
        new_points.push(Point::new(points[0].x, points[0].y, points[0].stroke_id));

        let interval = Self::path_length(points) / (n as f32 - 1.0);
        let mut d = 0.0;

        for i in 1..points.len() {
            if points[i].stroke_id == points[i - 1].stroke_id {
                let mut dist = geometry::euclidean_distance(&points[i - 1], &points[i]);
                if (d + dist) >= interval {
                    let mut first_point = &points[i - 1];
                    while (d + dist) >= interval {
                        let t = if dist != 0. {
                            ((interval - d) / dist).clamp(0.0, 1.0)
                        } else {
                            0.5
                        };
                        let nx = (1.0 - t) * first_point.x + t * points[i].x;
                        let ny = (1.0 - t) * first_point.y + t * points[i].y;
                        new_points.push(Point::new(nx, ny, points[i].stroke_id));

                        // update partial length
                        dist = d + dist - interval;
                        d = 0.0;
                        first_point = new_points.last().unwrap();
                    }
                    d = dist;
                } else {
                    d += dist;
                }
            }
        }
        // sometimes we fall a rounding-error short of adding the last point, so add it if so
        if new_points.len() == n - 1 {
            let last = new_points.last().unwrap();
            new_points.push(Point::new(last.x, last.y, last.stroke_id));
        }
        new_points
    }

    /// Computes the path length for an array of points
    fn path_length(points: &[Point]) -> f32 {
        let mut length = 0.0;
        for i in 1..points.len() {
            if points[i].stroke_id == points[i - 1].stroke_id {
                length += geometry::euclidean_distance(&points[i - 1], &points[i]);
            }
        }
        length
    }

    /// Scales point coordinates to the integer domain [0..MAXINT-1] x [0..MAXINT-1]
    fn transform_coordinates_to_integers(&mut self) {
        for p in &mut self.points {
            p.int_x = ((p.x + 1.0) / 2.0 * (MAX_INT_COORDINATES as f32 - 1.0)) as i32;
            p.int_y = ((p.y + 1.0) / 2.0 * (MAX_INT_COORDINATES as f32 - 1.0)) as i32;
        }
    }

    /// Constructs a Lookup Table that maps grip points to the closest point from the gesture path
    fn construct_lut(&mut self) {
        let mut table: Vec<Vec<usize>> = vec![vec![0; LUT_SIZE]; LUT_SIZE];
        for i in 0..LUT_SIZE {
            for j in 0..LUT_SIZE {
                let mut min_dist = i32::MAX;
                let mut idx_min: usize = 0;
                for (t, p) in self.points.iter().enumerate() {
                    let row = p.int_y / LUT_SCALE_FACTOR as i32;
                    let col = p.int_x / LUT_SCALE_FACTOR as i32;
                    let dr = row - i as i32;
                    let dc = col - j as i32;
                    let dist = dr * dr + dc * dc;
                    if dist < min_dist {
                        min_dist = dist;
                        idx_min = t;
                    }
                }
                table[i][j] = idx_min as usize;
            }
        }
        self.lut = Some(table);
    }
}