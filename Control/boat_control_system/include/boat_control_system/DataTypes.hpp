// Filename: Datatypes.hpp
// Author: Kieran Pereira
// Date: 10/11/2024
// Description: Header file containing shared data structures used across multiple scripts.

#ifndef DATATYPES_HPP
#define DATATYPES_HPP

namespace Datatypes {
    
    //System used to define coordinates using latitude and longitude
    struct Coordinate {
        float latitude;
        float longitude;
    };
}

#endif // DATATYPES_HPP