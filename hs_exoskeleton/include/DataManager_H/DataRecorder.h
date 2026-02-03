#ifndef DATA_RECORDER_H
#define DATA_RECORDER_H

//C system headers

//C++ standard library headers
#include <functional>
#include <vector>
#include <string>

//other libraries' headers
#include <Eigen/Core>

//project's headers
#include "TEXTIO.h"

using namespace std;
using namespace std::placeholders;
using namespace Eigen;

class DataRecorder
{
private:
	vector<function<void(void)>> rec;
	vector<function<void(void)>> RecEnd;
	vector<WriteDataIntoText*> vec_rec;

public:
	DataRecorder()
	{

	}

	~DataRecorder()
	{
		clear();
	}

	void clear()
	{
		ending();
	}

	template<class T>
	DataRecorder& addRecorder(T& obj, const string& path)
	{
		obj.setDataRecordingPath(path);
		rec.push_back(bind(&T::dataRecording, &obj));
		RecEnd.push_back(bind(&T::stopRecording, &obj));
		return *this;
	}

	template<class T>
	DataRecorder& addRecorder(const T* pVec, const string& path)
	{
		vec_rec.push_back(new WriteDataIntoText(path));
		rec.push_back(
			bind(
				(void(WriteDataIntoText::*)(const double*, int)) & WriteDataIntoText::operator(), 
				vec_rec.back(), 
				pVec->data(), 
				pVec->size()
			));
		RecEnd.push_back(bind(&WriteDataIntoText::ending, vec_rec.back()));
		return *this;
	}

	void operator()()
	{
		for (auto it = rec.begin(); it != rec.end(); ++it)
			(*it)();
	}

	void ending()
	{
		for (auto it = RecEnd.begin(); it != RecEnd.end(); ++it)
			(*it)();
		RecEnd.clear();
		rec.clear();

		if (!vec_rec.empty())
		{
			for (auto it = vec_rec.begin(); it != vec_rec.end(); ++it)
				delete (*it);
		}
		vec_rec.clear();
	}

};

#endif // !DATA_RECORDER_H