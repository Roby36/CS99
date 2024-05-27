
#pragma once
#include "utils.h"

/****************** cost / heuristic function interfaces ****************+*/

/****** len ******
 * 
 * This function calculates the absolute value, relative to the map, of the distance between two arbitrary grid coordinates.
 * 
 * Inputs:
 *      Two discrete, adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot
 *      Params struct containing the scaling factor s for the grid
 * Outputs:
 *      the resulting float-valued distance between (x_1,y_1) and (x_2,y_2), in map (not grid) units
 * 
 * This function will be applied on a series of adjacent points to calculate path lenths,
 * which will contribute to the path length linear component of the overall cost function (b-coefficient)
*/
float len(
    int x_1, int y_1, int x_2, int y_2,
    Params * params
); 

/****** g_image_riemann_sum ******
 * 
 * This function computes the Riemann sum, according to the (uncertainty) function g, 
 * between two arbitrary adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot
 * 
 * Inupts:
 *      Two discrete, adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot
 *      Params struct containing: 
 *          - scaling factors s and r for the grid and function g respectively
 *          - g_image containing the pre-computed values for g at r-steps resolution, stored in a 2-dimensional matrix
 *          - bounds xr_steps, yr_steps of the matrix containing g_image
 * 
 * Output:
 *      The contribution of taking a path integral of g, between (x_1,y_1) and (x_2,y_2), scaled to map units.
 * 
 * This function will be applied on a series of adjacent points in the domain of g to calculate path integrals numerically,
 * which will contribute to the uncertainty accumulation linear component of the overall cost function (a-coefficient)    
 */
float g_image_riemann_sum(
    int x_1, int y_1, int x_2, int y_2,
    Params * params
); 

/****** edge_cost ******
 * 
 * This is the resulting cost function combining linearly uncertainty accumulation and path lenth,
 * which are computed by g_image_riemann_sum and len respectively
 * 
 * Inputs:
 *      Two discrete, adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot
 *      Params struct containing the fixed environment parameters, passed as input for
 *          - g_image_riemann_sum
 *          - len
 *      Config struct containing the relative weights a and b for uncertainty accumulation and path length accordingly
 * 
 * Output:
 *      The resulting cost between the points (x_1,y_1) and (x_2,y_2) in the grid. Note that this is expressed in map units (floats)
 * 
 * This function will be passed as input for the A* search algorithm, serving as the main cost function for the graph search. 
 */
float edge_cost(
    int x_1, int y_1, int x_2, int y_2,
    Params * params,
    Config * config
); 

/****** heuristic ******
 * 
 * This is the main heuristic used for the basic (not homotopy class) version of A*.
 * It consists in aiming towards the goal as fast as possible, firstly by exploiting 
 * all the faster diagonal transitions available, and then finishing with the remaining straight ones.
 * For each step, we cancel the Riemann sum factor of the cost function, thereby guaranteeing
 * admissibility and swifter computation. 
 * 
 * Inputs:
 *      current point on the grid (x_n, y_n), and goal point (x_g, y_g)
 *      Config struct holding b coefficient of the cost function (the coefficient determining the weight for path length) 
 *      Params struct holding scaling factor s for the grid
 * 
 * Outputs:
 *      Resulting value of the heuristic between (x_n, y_n) and (x_g, y_g), 
 *          guaranteed to be lower than the lower cost path between the two points
 * 
 * This function is passed as parameter to the A* search algorithm, when this is merely looking for 
 * the lowest cost path (not the more general version keeping track of all the homotopy classes). The tightness of this
 * heuristic significantly accelerates the graph search procedure towards the goal
 */
float heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config
); 

/****** zero_heuristic ******
 * 
 * This is a trivial zero heuristic, used to turn A* into Dijkstra's algorithm in the case where
 * we are concerned in keeping track of all the possible homotopy classes.
 * In that case, we want to guarantee as much uniform and unbiased coverage as possible of the environment.
 * 
 * The dummy arguments make the function signature conform to the A* algorithm main function, 
 * thereby making the function reusable with different heuristic
 */
float zero_heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config
);

