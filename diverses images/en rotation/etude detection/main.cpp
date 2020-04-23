#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cassert>

#define _USE_MATH_DEFINES
#include <cmath>

using namespace std;

bool extract_uint16_t(string line, uint16_t * ret)
{
    stringstream ss(line);
    unsigned int a = 0, b = 0;
    if(ss >> a >> b)
    {
        *ret = ((a & 0xFF) << 8) | (b & 0xFF);
        return true;
    }

    *ret = 0;
    return false;
}

vector<vector<uint16_t>> extract_tab_images(const char * filename)
{
    string line;
    ifstream file(filename);

    vector<vector<uint16_t>> tab_images;

    while(getline(file, line))
    {
        if(line.find("NEW_IMAGE:") != string::npos)
            tab_images.push_back(vector<uint16_t>());
        else
        {
            uint16_t val_uint16_t;
            if(extract_uint16_t(line, &val_uint16_t))
                tab_images[tab_images.size() - 1].push_back(val_uint16_t);
        }
    }
    cout << "Found " << tab_images.size() << " images" << endl;
    unsigned int img_size = tab_images[0].size();

    // same size for all images ?
    bool same_size_problem = false;
    for(size_t i = 1 ; i < tab_images.size() ; ++i)
    {
        if(tab_images[i].size() != img_size)
        {
            same_size_problem = true;
            break;
        }
    }
    if(same_size_problem)
    {
        cout << "There is a problem about the size of ours images" << endl;
        for(size_t i = 0 ; i < tab_images.size() ; ++i)
            cout << "Image " << i << ": size: " << tab_images[i].size() << endl;
    }
    else
        cout << "Size: " << img_size << endl;


    return tab_images;
}

// RRRRRGGG GGGBBBBB
uint8_t extract_red(uint16_t pattern)
{
    return (pattern >> 11) & 31;
}

uint8_t extract_green(uint16_t pattern)
{
    return (pattern >> 5) & 63;
}

uint8_t extract_blue(uint16_t pattern)
{
    return pattern & 31;
}

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

void export_tab_images_to_csv(vector<vector<uint16_t>> tab_images)
{
    unsigned int img_size = tab_images[0].size();

    ofstream file(string("images.csv").c_str());
    file << "Nb images:," << tab_images.size() << endl;
    file << "Size:," << img_size << endl;

    file << "posX,";
    for(size_t j = 0 ; j < tab_images.size() ; ++j)
        file << "Image " << j + 1 << ",,,,";
    file << endl;

    file << ",";
    for(size_t j = 0 ; j < tab_images.size() ; ++j)
        file << "Pattern,Red,Green,Blue,Hue,Saturation,Value,";
    file << endl;

    for(unsigned int i = 0 ; i < img_size ; ++i)
    {
        file << i << ",";
        for(size_t j = 0 ; j < tab_images.size() ; ++j)
        {
            uint8_t r = extract_red(tab_images[j][i]);
            uint8_t g = extract_green(tab_images[j][i]);
            uint8_t b = extract_blue(tab_images[j][i]);
            double h, s, v;
            convert_rgb_to_hsv(h, s, v, r, g, b);
            file << tab_images[j][i] << ","
                << to_string(r) << ","
                << to_string(g) << ","
                << to_string(b) << ","
                << to_string(h) << ","
                << to_string(s) << ","
                << to_string(v) << ",";
        }

        file << endl;
    }

}

/// DEBUT DU CODE A METTRE DANS L'E-PUCK

#define IMAGE_BUFFER_SIZE               640
#define NO_RISE_FALL_FOUND_POS          IMAGE_BUFFER_SIZE + 1
#define GREEN_PIXEL_RISE_FALL_THRESHOLD (int16_t)(0.25 * 63)
#define THRESHOLD_BALL_COLOR_IN_GREEN   13
#define TAN_45_OVER_2_CONST             0.4142135679721832275390625f // in rad, fit for float

float compute_angle_ball(uint16_t ball_middle_pos)
{
    return atan((((float)ball_middle_pos / 320) - 1) * TAN_45_OVER_2_CONST) * 180.f / M_PI;
}

void detection_in_image(uint8_t * green_pixels)
{
    uint16_t last_fall_pos = NO_RISE_FALL_FOUND_POS;
    uint16_t sum;
    int16_t pixel_derivative;

    for(uint16_t i = 2 ; i < IMAGE_BUFFER_SIZE - 2 ; ++i)
    {
        pixel_derivative = (int16_t)green_pixels[i + 2] - (int16_t)green_pixels[i - 2];
        if(last_fall_pos < IMAGE_BUFFER_SIZE)   // if the beginning of a ball has been seen, we can look at the end of a ball
        {
            sum += green_pixels[i];
            if(pixel_derivative >= GREEN_PIXEL_RISE_FALL_THRESHOLD)
            {
                if(sum < THRESHOLD_BALL_COLOR_IN_GREEN * (i - last_fall_pos))
                {
                    float ball_angle = compute_angle_ball((last_fall_pos + i) >> 1);

                    cout << "ball is located in between: " << last_fall_pos << " and " << i << endl;
                    cout << "angle is: " << ball_angle;
                    last_fall_pos = NO_RISE_FALL_FOUND_POS;
                }
            }
        }
        if(pixel_derivative <= -GREEN_PIXEL_RISE_FALL_THRESHOLD)
        {
            last_fall_pos = i;
            sum = 0;
        }
    }
}


/// FIN DU CODE

void launch_detection_in_image(vector<uint16_t> & image)
{
    assert(image.size() == IMAGE_BUFFER_SIZE);
    uint8_t green_pixels[IMAGE_BUFFER_SIZE] = {0};

    for(size_t i = 0 ; i < image.size() ; ++i)
        green_pixels[i] = extract_green(image[i]);

    detection_in_image(green_pixels);
}

int main()
{
    vector<vector<uint16_t>> tab_images = extract_tab_images("balle_rouge_en_face_fond_blanc.txt");
    export_tab_images_to_csv(tab_images);

    //launch_detection_in_image(tab_images[0]);

    return 0;
}
