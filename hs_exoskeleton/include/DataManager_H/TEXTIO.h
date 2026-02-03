#ifndef TEXTIO_H
#define TEXTIO_H

//C system headers

//C++ standard library headers
#include <iostream>
#include <fstream>
#include <string>

//other libraries' headers
#include <Eigen/Core>

//project's headers

using namespace std;
using namespace Eigen;

class WriteDataIntoText
{
public:
    ofstream* outfile = nullptr;

    WriteDataIntoText()
    {
    }

    WriteDataIntoText(const string& path) : outfile(new ofstream(path.c_str()))
    {
    }

    ~WriteDataIntoText()
    {
        ending();
    }

    void setPath(const string& path)
    {
        ending();
        outfile = new ofstream(path.c_str());
    }

    template<class T>
    void operator()(const T& vec)
    {
        this->operator()(vec.data(), vec.size());
    }

    void operator()(const double* arr, int num)
    {
        for (int p = 0; p < num; ++p)*outfile << arr[p] << ' ';
        *outfile << '\n';
    }

    void operator()(const double& value)
    {
        *outfile << value << '\n';
    }

    void operator()(const int16_t& value)
    {
        *outfile << value << '\n';
    }

    void ending()
    {
        if (outfile != nullptr)
        {
            outfile->close();
            delete outfile;
            outfile = nullptr;
        }
    }
};

/*²ÎÊý¶ÁÈ¡*/
class ReadDataFromText
{
private:
    std::ifstream* file = nullptr;
public:
    ReadDataFromText(int n, double* arr, const char* PathOfText) :file(new std::ifstream(PathOfText))
    {
        if (this->operator()(arr, n) == 0)
        {
            for (int i = 0; i < n; i++) std::cout << arr[i] << ' ';
            std::cout << '\n';
        }

        file->close();
    }

    template<typename T>
    ReadDataFromText(T& arr, const string& path) :file(new std::ifstream(path.c_str())) {
        if (this->operator()(arr) == 0) {
            for (int i = 0; i < arr.size(); i++) std::cout << arr[i] << ' ';
            std::cout << '\n';
        }
        file->close();
    }

    ReadDataFromText(const string& path) :file(new std::ifstream(path))
    {

    }

    ReadDataFromText()
    {

    }

    ~ReadDataFromText()
    {
        if (file != nullptr)
        {
            file->close();
            delete file;
        }
    }
    void setPath(const string& TextPath)
    {
        if (file != nullptr)
        {
            file->close();
            delete file;
        }
        file = new std::ifstream(TextPath.c_str());
    }

    int operator()(double* arr, int num = 1)
    {
        if (file->eof() == true) {
            std::cout << "Nothing left!!!\n";
            return -1;
        }

        int i = 0;
        while (file->eof() == false && i < num) {
            *file >> arr[i];
            ++i;
        }
        if (i < num) {
            std::cout << "Lack of data!!!\n";
            return -1;
        }

        return 0;
    }

    template<typename T>
    int operator()(T& arr) {
        const auto& num = arr.size();
        if (file->eof() == true) {
            std::cout << "Nothing left!!!\n";
            return -1;
        }

        int i = 0;
        while (file->eof() == false && i < num) {
            *file >> arr[i];
            ++i;
        }
        if (i < num) {
            std::cout << "Lack of data!!!\n";
            return -1;
        }

        return 0;
    }
};
#endif // TEXTIO_H
