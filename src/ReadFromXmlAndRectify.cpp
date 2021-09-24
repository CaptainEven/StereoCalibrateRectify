#include"ReadFromXmlAndRectify.h"



int readParamsFromXml(const string& xml_path, const string& elem_name, vector<double>& params)
{
	XMLDocument doc;
	doc.LoadFile(xml_path.c_str());
	XMLElement* ptr_opencv_storage = doc.RootElement();
	if (!ptr_opencv_storage)
	{
		cout << "[Warning]: Parse the root node failed when reading "
			<< "[" + elem_name + "]" << endl;
		return -1;
	}

	XMLElement* ptr_rotate_left = ptr_opencv_storage->FirstChildElement(elem_name.c_str());
	if (ptr_rotate_left)
	{
		XMLElement* first_child = ptr_rotate_left->FirstChildElement();

		const char* content = "", *name = "";
		while (first_child)
		{
			name = first_child->Name();
			if (strcmp(name, "data") == 0)
			{
				content = first_child->GetText();
				break;
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}
		//cout << "params:\n" << string(content) << endl;

		// return left camera distortion coefficients
		vector<string> tokens;
		splitStr(string(content), tokens, ' ');
		params.reserve(tokens.size());

		for (const auto& token : tokens)
		{
			//cout << token << endl;
			params.push_back(atof(token.c_str()));
		}
	}

	return 0;
}

int parseDoubleStr(const string& content, const int& rows, const int& cols, cv::Mat& mat)
{
	if (content.size() == 0 || rows == 0 || cols == 0)
	{
		return -1;
	}

	mat.create(rows, cols, CV_64FC1);

	vector<string> tokens;
	splitStr(content, tokens, ' ');  // 字符串切分
	filterStr(tokens);  // 字符串数组预处理

	double val = 0.0;
	int x = 0, y = 0;
	for (int i = 0; i < tokens.size(); ++i)
	{
		const string& token = tokens[i];

		if ((token).find("e") != string::npos)  // 科学计数法
		{
			val = str2num(token);
		}
		else  // 一般浮点
		{
			val = atof(token.c_str());
		}
		//cout << val << " ";

		y = i / cols;
		x = i % cols;
		mat.at<double>(y, x) = val;
	}

	return 0;
}

int readStereoCalibFromXml(const string& xml_path,
	Mat& K1, Mat& dist1, 
	Mat& K2, Mat& dist2,
	Mat& R, Mat& T,
	Mat& R1, Mat& R2, 
	Mat& P1, Mat& P2,
	Mat& Q)
{
	XMLDocument doc;
	doc.LoadFile(xml_path.c_str());
	XMLElement* ptr_opencv_storage = doc.RootElement();
	if (!ptr_opencv_storage)
	{
		cout << "[Warning]: Parse the root node failed when reading " << endl;
		return -1;
	}

	/*Parsing*/
	int rows = 0, cols = 0;
	string content = "", name = "";
	vector<string> tokens;

	cout << "\nParsing K1...\n";
	XMLElement* ptr_K1 = ptr_opencv_storage->FirstChildElement("cameraMatrix1");
	if (ptr_K1)
	{
		XMLElement* first_child = ptr_K1->FirstChildElement();
		while (first_child)
		{
			name = string(first_child->Name());
			if (strcmp(name.c_str(), "data") == 0)
			{
				content = first_child->GetText();
				first_child = first_child->NextSiblingElement();
			}
			else if(strcmp(name.c_str(), "rows") == 0)
			{
				rows = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "cols") == 0)
			{
				cols = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}

		cout << "K1(string vector):" << content << endl;
	}
	parseDoubleStr(content, rows, cols, K1);
	cout << "\nK1 mat:\n" << K1 << endl;

	cout << "\nParsing K2...\n";
	XMLElement* ptr_K2 = ptr_opencv_storage->FirstChildElement("cameraMatrix2");
	if (ptr_K2)
	{
		XMLElement* first_child = ptr_K2->FirstChildElement();
		while (first_child)
		{
			name = string(first_child->Name());
			if (strcmp(name.c_str(), "data") == 0)
			{
				content = first_child->GetText();
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "rows") == 0)
			{
				rows = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "cols") == 0)
			{
				cols = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}

		cout << "K2(string vector):" << content << endl;
	}
	parseDoubleStr(content, rows, cols, K2);
	cout << "K2 mat:\n" << K2 << endl;

	cout << "\nParsing dist1...\n";
	XMLElement* ptr_dist1 = ptr_opencv_storage->FirstChildElement("distCoeffs1");
	if (ptr_dist1)
	{
		XMLElement* first_child = ptr_dist1->FirstChildElement();
		while (first_child)
		{
			name = string(first_child->Name());
			if (strcmp(name.c_str(), "data") == 0)
			{
				content = first_child->GetText();
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "rows") == 0)
			{
				rows = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "cols") == 0)
			{
				cols = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}

		cout << "Dist1(string vector):" << content << endl;
	}
	parseDoubleStr(content, rows, cols, dist1);
	cout << "\nDist1 mat:\n" << dist1 << endl;

	cout << "\nParsing dist2...\n";
	XMLElement* ptr_dist2 = ptr_opencv_storage->FirstChildElement("distCoeffs2");
	if (ptr_dist2)
	{
		XMLElement* first_child = ptr_dist2->FirstChildElement();
		while (first_child)
		{
			name = string(first_child->Name());
			if (strcmp(name.c_str(), "data") == 0)
			{
				content = first_child->GetText();
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "rows") == 0)
			{
				rows = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "cols") == 0)
			{
				cols = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}

		cout << "Dist2(string vector):" << content << endl;
	}
	parseDoubleStr(content, rows, cols, dist2);
	cout << "\nDist2 mat:\n" << dist2 << endl;

	cout << "\nParsing R...\n";
	XMLElement* ptr_R = ptr_opencv_storage->FirstChildElement("R");
	if (ptr_R)
	{
		XMLElement* first_child = ptr_R->FirstChildElement();
		while (first_child)
		{
			name = string(first_child->Name());
			if (strcmp(name.c_str(), "data") == 0)
			{
				content = first_child->GetText();
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "rows") == 0)
			{
				rows = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "cols") == 0)
			{
				cols = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}

		cout << "R(string vector):" << content << endl;
	}
	parseDoubleStr(content, rows, cols, R);
	cout << "R mat:\n" << R << endl;

	cout << "\nParsing T...\n";
	XMLElement* ptr_T = ptr_opencv_storage->FirstChildElement("T");
	if (ptr_T)
	{
		XMLElement* first_child = ptr_T->FirstChildElement();
		while (first_child)
		{
			name = string(first_child->Name());
			if (strcmp(name.c_str(), "data") == 0)
			{
				content = first_child->GetText();
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "rows") == 0)
			{
				rows = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "cols") == 0)
			{
				cols = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}

		cout << "T(string vector):" << content << endl;
	}
	parseDoubleStr(content, rows, cols, T);
	cout << "T mat:\n" << T << endl;

	cout << "\nParsing R1...\n";
	XMLElement* ptr_R1 = ptr_opencv_storage->FirstChildElement("R1");
	if (ptr_R1)
	{
		XMLElement* first_child = ptr_R1->FirstChildElement();
		while (first_child)
		{
			name = string(first_child->Name());
			if (strcmp(name.c_str(), "data") == 0)
			{
				content = first_child->GetText();
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "rows") == 0)
			{
				rows = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "cols") == 0)
			{
				cols = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}

		cout << "R1(string vector):" << content << endl;
	}
	parseDoubleStr(content, rows, cols, R1);
	cout << "R1 mat:\n" << R1 << endl;

	cout << "\nParsing R2...\n";
	XMLElement* ptr_R2 = ptr_opencv_storage->FirstChildElement("R2");
	if (ptr_R2)
	{
		XMLElement* first_child = ptr_R2->FirstChildElement();
		while (first_child)
		{
			name = string(first_child->Name());
			if (strcmp(name.c_str(), "data") == 0)
			{
				content = first_child->GetText();
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "rows") == 0)
			{
				rows = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "cols") == 0)
			{
				cols = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}

		cout << "R2(string vector):" << content << endl;
	}
	parseDoubleStr(content, rows, cols, R2);
	cout << "R2 mat:\n" << R2 << endl;

	cout << "\nParsing P1...\n";
	XMLElement* ptr_P1 = ptr_opencv_storage->FirstChildElement("P1");
	if (ptr_P1)
	{
		XMLElement* first_child = ptr_P1->FirstChildElement();
		while (first_child)
		{
			name = string(first_child->Name());
			if (strcmp(name.c_str(), "data") == 0)
			{
				content = first_child->GetText();
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "rows") == 0)
			{
				rows = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "cols") == 0)
			{
				cols = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}

		cout << "P1(string vector):" << content << endl;
	}
	parseDoubleStr(content, rows, cols, P1);
	cout << "P1 mat:\n" << P1 << endl;

	cout << "\nParsing P2...\n";
	XMLElement* ptr_P2 = ptr_opencv_storage->FirstChildElement("P2");
	if (ptr_P2)
	{
		XMLElement* first_child = ptr_P2->FirstChildElement();
		while (first_child)
		{
			name = string(first_child->Name());
			if (strcmp(name.c_str(), "data") == 0)
			{
				content = first_child->GetText();
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "rows") == 0)
			{
				rows = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "cols") == 0)
			{
				cols = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}

		cout << "P2(string vector):" << content << endl;
	}
	parseDoubleStr(content, rows, cols, P2);
	cout << "P2 mat:\n" << P2 << endl;

	cout << "\nParsing Q...\n";
	XMLElement* ptr_Q = ptr_opencv_storage->FirstChildElement("Q");
	if (ptr_Q)
	{
		XMLElement* first_child = ptr_Q->FirstChildElement();
		while (first_child)
		{
			name = string(first_child->Name());
			if (strcmp(name.c_str(), "data") == 0)
			{
				content = first_child->GetText();
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "rows") == 0)
			{
				rows = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else if (strcmp(name.c_str(), "cols") == 0)
			{
				cols = atoi(first_child->GetText());
				first_child = first_child->NextSiblingElement();
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}

		cout << "Q(string vector):" << content << endl;
	}
	parseDoubleStr(content, rows, cols, Q);
	cout << "Q mat:\n" << Q << endl;

	return 0;
}

int readLeftDistXml(const string& xml_path, vector<float>& l_dists)
{
	XMLDocument doc;
	doc.LoadFile(xml_path.c_str());
	XMLElement* ptr_opencv_storage = doc.RootElement();
	if (!ptr_opencv_storage)
	{
		cout << "Read root node failed.\n";
		return -1;
	}

	XMLElement* ptr_dist_left = ptr_opencv_storage->FirstChildElement("distCoeffL");
	if (ptr_dist_left)
	{
		XMLElement* first_child = ptr_dist_left->FirstChildElement();

		const char* content = "", *name = "";
		while (first_child)
		{
			name = first_child->Name();
			if (strcmp(name, "data") == 0)
			{
				content = first_child->GetText();
				break;
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}
		cout << "Left distortion coefficients:\n" << string(content) << endl;

		// return left camera distortion coefficients
		l_dists.reserve(5);
		vector<string> tokens;
		splitStr(string(content), tokens, ' ');

		for (auto& token : tokens)
		{
			//cout << token << endl;
			l_dists.push_back((float)atof(token.c_str()));
		}
	}
	else
	{
		cout << "Read Node [distCoeffL] failed.\n";
		return -1;
	}

	return 0;
}


int readRightDistXml(const string& xml_path, vector<float>& r_dists)
{
	XMLDocument doc;
	doc.LoadFile(xml_path.c_str());
	XMLElement* ptr_opencv_storage = doc.RootElement();
	if (!ptr_opencv_storage)
	{
		cout << "Read root node failed.\n";
		return -1;
	}

	XMLElement* ptr_dist_right = ptr_opencv_storage->FirstChildElement("distCoeffR");
	if (ptr_dist_right)
	{
		XMLElement* first_child = ptr_dist_right->FirstChildElement();

		const char* content = "", *name = "";
		while (first_child)
		{
			name = first_child->Name();
			if (strcmp(name, "data") == 0)
			{
				content = first_child->GetText();
				break;
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}
		cout << "Right distortion coefficients:\n" << string(content) << endl;

		// return right camera distortion coefficients
		r_dists.reserve(5);
		vector<string> tokens;
		splitStr(string(content), tokens, ' ');

		for (auto& token : tokens)
		{
			//cout << token << endl;
			r_dists.push_back((float)atof(token.c_str()));
		}
	}
	else
	{
		cout << "Read Node [distCoeffR] failed.\n";
		return -1;
	}

	return 0;
}


int readLeftRotateXml(const string& xml_path, vector<float>& l_rotate)
{
	XMLDocument doc;
	doc.LoadFile(xml_path.c_str());
	XMLElement* ptr_opencv_storage = doc.RootElement();
	if (!ptr_opencv_storage)
	{
		cout << "Read root node failed.\n";
		return -1;
	}

	XMLElement* ptr_rotate_left = ptr_opencv_storage->FirstChildElement("Rl");
	if (ptr_rotate_left)
	{
		XMLElement* first_child = ptr_rotate_left->FirstChildElement();

		const char* content = "", *name = "";
		while (first_child)
		{
			name = first_child->Name();
			if (strcmp(name, "data") == 0)
			{
				content = first_child->GetText();
				break;
			}
			else
			{
				first_child = first_child->NextSiblingElement();
			}
		}
		cout << "Right roration matrix:\n" << string(content) << endl;

		// return left camera distortion coefficients
		vector<string> tokens;
		splitStr(string(content), tokens, ' ');
		l_rotate.reserve(tokens.size());

		for (auto& token : tokens)
		{
			//cout << token << endl;
			l_rotate.push_back((float)atof(token.c_str()));
		}
	}

	return 0;
}




