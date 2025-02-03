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

/// Main function of the $P recognizer.
/// Classifies a candidate gesture against a set of training samples.
/// Returns the class of the closest neighbor in the training set.
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
fn greedy_cloud_match(points1: &[Point], points2: &[Point]) -> f32 {
    // the two clouds should have the same number of points by now
    let n = points1.len();

    // controls the number of greedy search trials (eps is in [0..1])
    let eps = 0.5;

    let step = (n as f32).powf(1.0 - eps).floor() as usize;
    let mut min_distance = f32::MAX;
    for i in (0..n).step_by(step) {
        // match points1 --> points2 starting with index point i
        let dist1 = cloud_distance(points1, points2, i);
        // match points2 --> points1 starting with index point i
        let dist2 = cloud_distance(points2, points1, i);
        min_distance = min_distance.min(dist1).min(dist2);
    }
    min_distance
}

/// Computes the distance between two point clouds by performing a minimum-distance greedy matching
/// starting with point startIndex
fn cloud_distance(points1: &[Point], points2: &[Point], start_index: usize) -> f32 {
    // the two clouds should have the same number of points by now
    let n = points1.len();
    // matched[i] signals whether point i from the 2nd cloud has been already matched
    let mut matched = vec![false; n];
    // computes the sum of distances between matched points (i.e., the distance between the two clouds)
    let mut sum = 0.0;
    let mut i = start_index;
    loop {
        let mut index = 0;
        let mut min_dist = f32::MAX;
        for j in 0..n {
            if !matched[j] {
                let dist = geometry::euclidean_distance(&points1[i], &points2[j]);
                if dist < min_dist {
                    min_dist = dist;
                    index = j;
                }
            }
        }
        // point index from the 2nd cloud is matched to point i from the 1st cloud
        matched[index] = true;
        let weight = 1.0 - (((i - start_index + n) % n) as f32 / n as f32);
        // weight each distance with a confidence coefficient that decreases from 1 to 0
        sum += weight * min_dist;
        i = (i + 1) % n;
        if i == start_index {
            break;
        }
    }
    sum
}