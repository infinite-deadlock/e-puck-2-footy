#include <iostream>
#include <fstream>
#include <math.h>
#include <cassert>

using namespace std;

void convert_rgb_to_hsv(double & h, double & s, double & v, uint8_t r_int, uint8_t g_int, uint8_t b_int)
{
    // adapté de https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
    // consulté le 27.04.2020
    // David H
    double min, max, delta;
    double r(r_int), g(g_int), b(b_int);
    r /= 31.;
    g /= 63.;
    b /= 31.;

    min = r < g ? r: g;
    min = min < b ? min: b;

    max = r > g ? r: g;
    max = max > b ? max: b;

    v = max;
    delta = max - min;

    if(delta < 0.00001)
    {
        s = 0;
        h = 0;
        return;
    }
    if(max > 0.0)
        s = delta / max;
    else
    {
        s = 0.0;
        h = NAN;
        return;
    }
    if(r >= max)
        h = (g - b) / delta;
    else if(g >= max)
        h = 2.0 + (b - r) / delta;
    else
        h = 4.0 + (r - g) / delta;

    h *= 60.0;

    if(h < 0.0)
        h += 360.0;

    h /= 360;
}

bool check_ball_presence(uint8_t r, uint8_t g, uint8_t b)
{
    double h,s,v;
    convert_rgb_to_hsv(h,s,v,r,g,b);

    return (s > 0.5 && (h < 0.05 || h > 0.9));
}

//#define NB_COLORS 0xFFFF
#define MAX_VALUE_CASE          8
#define MASK_LOOKUP_CASE_LENGTH 3
#define NB_LOOKUP_PRESENCE_CASE (0xFFFF >> MASK_LOOKUP_CASE_LENGTH)

int main()
{
    static uint8_t lookup_check_ball_presence[NB_LOOKUP_PRESENCE_CASE] = {0};
    for(unsigned int i = 0 ; i < NB_LOOKUP_PRESENCE_CASE ; i++)
    {
        assert(lookup_check_ball_presence[i] == 0);
        for(unsigned int j = 0 ; j < MAX_VALUE_CASE ; ++j)
        {
            uint16_t tmp = (i << MASK_LOOKUP_CASE_LENGTH) | j;
            uint8_t r, g, b;
            r = (tmp >> 11) & 31;
            g = (tmp >> 5) & 63;
            b = tmp & 31;

            //cout << "tmp: " << tmp << " r: " << (int)r << " g: " << (int)g << " b: " << (int)b << " " << (int)check_ball_presence(r, g, b) << endl;
            if(check_ball_presence(r, g, b))
                lookup_check_ball_presence[i] |= 1 << j;
            else
                lookup_check_ball_presence[i] |= 0 << j;
        }
    }

    ofstream file("out_code.txt");
    file << "static const uint8_t lookup_check_ball_presence[NB_LOOKUP_PRESENCE_CASE] = {" << endl;

    int element_per_line = 0;
    for(unsigned int i = 0 ; i < NB_LOOKUP_PRESENCE_CASE ; ++i)
    {
        if(lookup_check_ball_presence[i] < 10)
            file << " ";
        if(lookup_check_ball_presence[i] < 100)
            file << " ";
        file << (int)lookup_check_ball_presence[i];

        ++element_per_line;
        if(i < NB_LOOKUP_PRESENCE_CASE - 1)
        {
            if(element_per_line > 13)
            {
                file << "," << endl;
                element_per_line = 0;
            }
            else
                file << ", ";
        }
    }

    file << "};" << endl;

    return 0;
}
