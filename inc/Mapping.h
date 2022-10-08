//
// Created by sk8 on 07.10.2022.
//

#ifndef QUADRUPEDV2_MAPPING_H
#define QUADRUPEDV2_MAPPING_H

#pragma once

/**
 * Converts one range to another. Similar to Arduino map function.
 * @param x Variable which range is converting
 * @param in_min Variable min bound.
 * @param in_max  Variable max bound.
 * @param out_min Targeted min bound.
 * @param out_max Targeted max bound.
 * @return Returns variable in targeted bound.
 */
template<typename type>
type map(type x, type in_min, type in_max, type out_min, type out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif //QUADRUPEDV2_MAPPING_H
