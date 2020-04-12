#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

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
        file << "Pattern,Red,Green,Blue,";
    file << endl;

    for(unsigned int i = 0 ; i < img_size ; ++i)
    {
        file << i << ",";
        for(size_t j = 0 ; j < tab_images.size() ; ++j)
            file << tab_images[j][i] << ","
                << to_string(extract_red(tab_images[j][i])) << ","
                << to_string(extract_green(tab_images[j][i])) << ","
                << to_string(extract_blue(tab_images[j][i])) << ",";
        file << endl;
    }

}

int main()
{
    vector<vector<uint16_t>> tab_images = extract_tab_images("balle_rouge_en_face_fond_blanc.txt");
    export_tab_images_to_csv(tab_images);

    return 0;
}
