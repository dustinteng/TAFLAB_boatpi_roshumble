// Filename: Coordinate_Calculations.h
// Author: Kieran Pereira
// Last Modified: 10/11/2024
// Description: A script containing all relevant functions to do coordinate calculations (including tacking path planning)

#ifndef TACKING_PATH_PLANNER_H
#define TACKING_PATH_PLANNER_H

#include <vector>
#include <string>
#include "DataTypes.hpp"

using namespace std;

// Class Definition
class CoordinateCalculations 
{
public:

    /**
    @brief A function used to calculate the optimal sail angle given the wind direction
    @param wind_direction A float value indicating the given wind direction taken in reference to the boat
    @return A float value of the optimal sail angle
    **/
    float calculate_optimal_sail_angle(float wind_direction);


    /**
    @brief A function used to convert find the position of a waypoint, given the boats current position, the bearing and distance to the boat
    @param curr_position The coordinate location of the boat
    @param angle The angle bearing between the boat and the new position
    @param distance The distance between the boat and the new waypoint
    @return new_position The coordinates of the new position expressed as a lat, long position
    **/
    Datatypes::Coordinate convert_to_position(Datatypes::Coordinate curr_position, float angle, float distance);


    /**
    @brief A function used to calculate the distance between two coordinates
    @param coord1 The first co-ordinate
    @param coord2 The second co-ordinate
    @return A float value of the distance between the two coordinates in meters
    **/
    float calculate_distance(Datatypes::Coordinate coord1, Datatypes::Coordinate coord2);


    /**
    @brief Calculates the initial bearing (compass direction) from the current coordinate to the target coordinate.
    This function calculates the bearing between two geographic coordinates using trigonometric functions. 
    The bearing is returned in degrees, representing the direction from the current position to the target.
    @param curr_coord The current coordinate (latitude and longitude).
    @param tar_coord The target coordinate (latitude and longitude).
    @return float The bearing in degrees from the current coordinate to the target coordinate, in the range [0, 360).
    @note The function uses spherical trigonometry to determine the bearing.
    **/
    float calculate_bearing(Datatypes::Coordinate start, Datatypes::Coordinate end);

    /**
    @brief Calculates the directional bearing required to turn towards a target waypoint
    This function computes the bearing difference between the current heading and the target waypoint. 
    It determines whether a right or left turn is needed and returns the direction and angle required.
    @param target_waypoint The coordinate of the target waypoint to navigate towards.
    @return std::pair<std::string, float> A pair containing the direction ("right" or "left") and the turn angle in degrees.
    @note The function assumes that the current heading and coordinates are obtained from other methods in the class.
     **/
    std::pair<string, float> calculate_directional_bearing(Datatypes::Coordinate target_waypoint);
    

    /**
    @brief A temporary function used to check the difference between the current heading and angle to wind
    need to determine if this function returns the same as the wind sensor (which it should)
    @return float A float value of the angle between the wind and current heading
    @note The function assumes that the current heading and coordinates are obtained from other methods in the class.
     **/
    float calculate_angle_to_wind();



    void plan_path(const Datatypes::Coordinate& curr_position, const Datatypes::Coordinate& next_waypoint);

    // Static Method to access single instance
    static CoordinateCalculations& getInstance();

private:

    // Defining a constructor for a singleton class
    CoordinateCalculations();

    //Destructor (Added for completeness)
    ~CoordinateCalculations() = default;


    static const unsigned int max_upwind_angle = 45;
    bool tack_status = false;

};

#endif // TACKING_PATH_PLANNER_H