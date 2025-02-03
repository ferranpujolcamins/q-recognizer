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

use crate::{geometry, gesture::Gesture, point::Point};
use std::f32;

/// Main function of the $P+ recognizer.
/// Classifies a candidate gesture against a set of training samples.
/// Returns the class of the closest neighbor in the template set.
pub fn classify(candidate: &Gesture, training_set: &[Gesture]) -> String {
    let mut min_distance = f32::MAX;
    let mut gesture_class = String::new();
    for template in training_set {
        let dist = greedy_cloud_match(&candidate.points, &template.points);
        if dist < min_distance {
            min_distance = dist;
            gesture_class = template.name.clone();
        }
    }
    gesture_class
}

/// Implements greedy search for a minimum-distance matching between two point clouds
/// using local shape descriptors (theta turning angles).
fn greedy_cloud_match(points1: &[Point], points2: &[Point]) -> f32 {
    // should be pre-processed in the Gesture class
    let theta1 = compute_local_shape_descriptors(points1);
    // should be pre-processed in the Gesture class
    let theta2 = compute_local_shape_descriptors(points2);
    let d1 = cloud_distance(points1, &theta1, points2, &theta2);
    let d2 = cloud_distance(points2, &theta2, points1, &theta1);
    d1.min(d2)
}

/// Computes the distance between two point clouds 
/// using local shape descriptors (theta turning angles).
fn cloud_distance(
    points1: &[Point],
    theta1: &[f32],
    points2: &[Point],
    theta2: &[f32]
) -> f32 {
    let mut matched = vec![false; points2.len()];
    let mut sum = 0.0;
    let mut index = 0;

    for i in 0..points1.len() {
        sum += get_closest_point_from_cloud(&points1[i], theta1[i], points2, theta2, &mut index);
        matched[index] = true;
    }
    for i in 0..points2.len() {
        if !matched[i] {
            sum += get_closest_point_from_cloud(&points2[i], theta2[i], points1, theta1, &mut index);
        }
    }
    sum
}

/// Searches for the point from point-cloud cloud that is closest to point p.
fn get_closest_point_from_cloud(
    p: &Point,
    theta: f32,
    cloud: &[Point],
    theta_cloud: &[f32],
    index_min: &mut usize
) -> f32 {
    let mut min = f32::MAX;
    *index_min = 0;
    for i in 0..cloud.len() {
        let dx = geometry::sqr_euclidean_distance(p, &cloud[i]);
        let dtheta = theta - theta_cloud[i];
        let dist = (dx + dtheta * dtheta).sqrt();
        if dist < min {
            min = dist;
            *index_min = i;
        }
    }
    min
}

/// Computes local shape descriptors (theta turning angles) at each point on the gesture path.
pub fn compute_local_shape_descriptors(points: &[Point]) -> Vec<f32> {
    let n = points.len();
    let mut theta = vec![0.0; n];
    for i in 1..(n - 1) {
        theta[i] = short_angle(&points[i - 1], &points[i], &points[i + 1]) / std::f32::consts::PI;
    }
    theta
}

/// Computes the smallest turning angle between vectors (a,b) and (b,c) in radians in the interval [0..PI].
fn short_angle(a: &Point, b: &Point, c: &Point) -> f32 {
    let length_ab = geometry::euclidean_distance(a, b);
    let length_bc = geometry::euclidean_distance(b, c);
    if (length_ab * length_bc).abs() <= f32::EPSILON {
        return 0.0;
    }
    // compute cosine of the angle between vectors (a,b) and (b,c)
    let dot = (b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y);
    let cos_angle = dot / (length_ab * length_bc);
    
    // deal with special cases near limits of the [-1,1] interval
    if cos_angle <= -1.0 {
        std::f32::consts::PI
    } else if cos_angle >= 1.0 {
        0.0
    } else {
        // return the angle between vectors (a,b) and (b,c) in the interval [0,PI]
        cos_angle.acos()
    }
}