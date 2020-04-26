#include <iostream>
#include <fstream>
#include <math.h>

using namespace std;

void convert_rgb_to_hsv(double & h, double & s, double & v, uint8_t r_int, uint8_t g_int, uint8_t b_int)
{
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

    return (s > 0.5 && (h < 0.1 || h > 0.9));
}

#define NB_COLORS 0xFFFF

int main()
{
    static bool lookup_check_ball_presence[NB_COLORS];
    for(unsigned int i = 0 ; i < NB_COLORS ; i++)
    {
        uint8_t r, g, b;
        r = (i >> 11) & 31;
        g = (i >> 5) & 63;
        b = i & 31;

        lookup_check_ball_presence[i] = check_ball_presence(r, g, b);

        //cout << "r: " << (int)r << " g: " << (int)g << " b: " << (int)b << " presence: " << lookup_check_ball_presence[i] << endl;
    }

    ofstream file("out_code.txt");
    file << "static const bool lookup_check_ball_presence[NB_COLORS] = {" << endl;

    int element_per_line = 0;
    for(unsigned int i = 0 ; i < NB_COLORS ; ++i)
    {
        if(lookup_check_ball_presence[i])
            file << "true";
        else
            file << "false";

        ++element_per_line;
        if(i < NB_COLORS - 1)
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
