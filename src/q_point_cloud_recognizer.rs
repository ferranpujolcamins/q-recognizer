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

use crate::{geometry::sqr_euclidean_distance, gesture::{Gesture, LUT_SCALE_FACTOR}, point::Point};

pub struct QParameters {
    // $Q's two major optimization layers (Early Abandoning and Lower Bounding)
    // can be activated / deactivated as desired
    pub use_early_abandoning: bool,
    pub use_lower_bounding: bool
}

impl Default for QParameters {
    fn default() -> Self {
        QParameters {
            use_early_abandoning: true,
            use_lower_bounding: true,
        }
    }
}

/// Main function of the $Q recognizer.
/// Classifies a candidate gesture against a set of templates.
/// Returns the class of the closest neighbor in the template set.
pub fn classify(candidate: &Gesture, template_set: &[Gesture], params: &QParameters) -> String {
    let mut best_class = String::new();
    let mut min_dist = f32::MAX;
    for template in template_set {
        let d = greedy_cloud_match(candidate, template, min_dist, params);
        if d < min_dist {
            min_dist = d;
            best_class = template.name.clone();
        }
    }
    best_class
}

/// Implements greedy search for a minimum-distance matching between two point clouds.
/// Implements Early Abandoning and Lower Bounding (LUT) optimizations.
fn greedy_cloud_match(gesture1: &Gesture, gesture2: &Gesture, mut min_so_far: f32, params: &QParameters) -> f32 {
    // the two clouds should have the same number of points by now
    let n = gesture1.points.len();
    // controls the number of greedy search trials (eps is in [0..1])
    let eps = 0.5;
    let step = (n as f32).powf(1.0 - eps).floor() as usize;

        if params.use_lower_bounding {
            // direction of matching: gesture1 --> gesture2
            let lb1 = compute_lower_bound(&gesture1.points, &gesture2.points, gesture2.lut.as_ref().unwrap(), step);
            // direction of matching: gesture2 --> gesture1
            let lb2 = compute_lower_bound(&gesture2.points, &gesture1.points, &gesture1.lut.as_ref().unwrap(), step);

            let mut i = 0;
            let mut index_lb = 0;
            while i < n {
                if lb1[index_lb] < min_so_far {
                    // direction of matching: gesture1 --> gesture2 starting with index point i
                    min_so_far = min_so_far.min(cloud_distance(&gesture1.points, &gesture2.points, i, min_so_far, params));
                }
                if lb2[index_lb] < min_so_far {
                    // direction of matching: gesture2 --> gesture1 starting with index point i
                    min_so_far = min_so_far.min(cloud_distance(&gesture2.points, &gesture1.points, i, min_so_far, params));
                }

                index_lb += 1;
                i += step;
            }
        } else {
            for i in (0..n).step_by(step) {
                // direction of matching: gesture1 --> gesture2 starting with index point i
                min_so_far = min_so_far.min(cloud_distance(&gesture1.points, &gesture2.points, i, min_so_far, params));
                // direction of matching: gesture2 --> gesture1 starting with index point i   
                min_so_far = min_so_far.min(cloud_distance(&gesture2.points, &gesture1.points, i, min_so_far, params));
            }
        }

    min_so_far
}

/// Computes lower bounds for each starting point and the direction of matching from points1 to points2 
fn compute_lower_bound(
    points1: &[Point],
    points2: &[Point],
    lut: &Vec<Vec<usize>>,
    step: usize
) -> Vec<f32> {
    let n = points1.len();
    let mut lb = vec![0.0; n / step as usize + 1];
    let mut sat = vec![0.0; n];

    for i in 0..n {
        let index = lut[points1[i].int_y as usize / LUT_SCALE_FACTOR]
            [points1[i].int_x as usize / LUT_SCALE_FACTOR];
        let dist = sqr_euclidean_distance(&points1[i], &points2[index]);
        sat[i] = if i == 0 { dist } else { sat[i - 1] + dist };
        lb[0] += (n - i) as f32 * dist;
    }

    let mut i = step;
    let mut index_lb = 1;
    while i < n {
        lb[index_lb] = lb[0] + (i as f32)*sat[n-1] - (n as f32)*sat[i-1];

        index_lb += 1;
        i += step;
    }
    lb
}

/// Computes the distance between two point clouds by performing a minimum-distance greedy matching
/// starting with point startIndex
fn cloud_distance(points1: &[Point], points2: &[Point], start_index: usize, min_so_far: f32, params: &QParameters) -> f32 {
    // the two point clouds should have the same number of points by now
    let n = points1.len();
    // stores point indexes for points from the 2nd cloud that haven't been matched yet
    let mut indexes_not_matched: Vec<usize> = (0..n).collect();

    // computes the sum of distances between matched points (i.e., the distance between the two clouds)
    let mut sum = 0.0;
    // start matching with point startIndex from the 1st cloud
    let mut i = start_index;
    // implements weights, decreasing from n to 1
    let mut weight = n;
    // indexes the indexesNotMatched[..] array of points from the 2nd cloud that are not matched yet
    let mut index_not_matched = 0;

    loop {
        let mut index = 0;
        let mut min_distance = f32::MAX;
        for j in index_not_matched..n {
            let dist = sqr_euclidean_distance(&points1[i], &points2[indexes_not_matched[j]]);
            if dist < min_distance {
                min_distance = dist;
                index = j;
            }
        }
        // point indexesNotMatched[index] of the 2nd cloud is now matched to point i of the 1st cloud
        indexes_not_matched[index] = indexes_not_matched[index_not_matched];
        // weight each distance with a confidence coefficient that decreases from n to 1
        sum += (weight as f32) * min_distance;
        weight -= 1;

        
        if params.use_early_abandoning && sum >= min_so_far {
            return sum;
        }

        // advance to the next point in the 1st cloud
        i = (i + 1) % n;
        // update the number of points from the 2nd cloud that haven't been matched yet
        index_not_matched += 1;

        if i == start_index {
            break;
        }
    }
    sum
}