// Filename: SailControl.cpp
// Author: Kieran Pereira
// Date: 10/11/2024
// Description: Implementation file for autonomously controlling the sail

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <Arduino.h>

#include "Sail_Control.hpp"
#include "TAF_AS5600.h"
#include "Boat_SM.h"
#include "Boat_steer.h"


std::vector<SailData> loadSailData(const std::string& filename)
{
    std::vector<SailData> sailData;
    std::ifstream file(filename);
    std::string line;

    if(!file.is_open())
    {
        throw std::runtime_error("Could not open file");
    }

    // Skip the header line
    std::getline(file, line);

    // Read each line of the file
    while(std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string item;
        SailData data;

        // Read wind angle
        std::getline(ss, item, ',');
        data.windAngle = std::stoi(item);

        // Read optimal sail position
        std::getline(ss, item, ',');
        data.optimalSailPosition = std::stoi(item);

        sailData.push_back(data);
    }

    file.close();
    return sailData;
}

int interpolateSailPosition(int windAngle, const SailData& lower, const SailData& upper)
{
    // Linear interpolation formula
    return lower.optimalSailPosition + (windAngle - lower.windAngle) * 
           (upper.optimalSailPosition - lower.optimalSailPosition) / 
           (upper.windAngle - lower.windAngle);
}

int getOptimalSailPosition(int windAngle, const std::vector<SailData>& sailData)
{
    // Check if exact match exists
    for(const auto& data : sailData)
    {
        if (data.windAngle == windAngle) {
            return data.optimalSailPosition;
        }
    }

    // If no exact match, find the closest two angles for interpolation
    for(size_t i = 0; i < sailData.size() - 1; ++i)
    {
        if(sailData[i].windAngle < windAngle && sailData[i + 1].windAngle > windAngle)
        {
            return interpolateSailPosition(windAngle, sailData[i], sailData[i + 1]);
        }
    }
    return -1;
}

void sail_control_task(void* parameter)
{
    while(1)
    {
        if(get_curr_state() == AUTO)
        {
            int windAngle = get_avg_angle();
            int optimalSailPos = getOptimalSailPosition(windAngle,sailData);
            if(optimalSailPos != -1)
            {
                set_sail_servo(optimalSailPos);
            }else{
                std::cout<<"No Valid Sail Position for this wind Angle"<<std::endl;
            }
        }
        // vTaskDelay(STEERING_DELAY / portTICK_PERIOD_MS);
    }
}
