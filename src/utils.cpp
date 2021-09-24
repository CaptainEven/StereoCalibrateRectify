#include"utils.h"


void splitStr(const string& s, vector<string>& tokens, const char& delim = ' ')
{
	tokens.clear();
	size_t last_pos = s.find_first_not_of(delim, 0);
	size_t pos = s.find(delim, last_pos);
	while (last_pos != string::npos)
	{
		tokens.emplace_back(s.substr(last_pos, pos - last_pos));
		last_pos = s.find_first_not_of(delim, pos);
		pos = s.find(delim, last_pos);
	}
}


void replaceStr(const string& src_str,
	const string &old_str, const string& new_str,
	string& ret,
	int count = -1)
{
	ret = string(src_str);  // string的拷贝构造

	size_t pos = 0;
	int l_count = 0;
	if (-1 == count)  // replace all
	{
		count = (int)ret.size();  // max size
	}
	while ((pos = ret.find(old_str, pos)) != string::npos)
	{
		ret.replace(pos, old_str.size(), new_str);
		if (++l_count >= count)
		{
			break;
		}

		pos += new_str.size();
	}
}

int filterStr(vector<string>& tokens)
{
	// 字符串数组预处理
	if (tokens.size() == 0)
	{
		std::printf("Empty string vector.\n");
		return -1;
	}

	auto it = tokens.begin();
	while (it != tokens.end())
	{
		if (strcmp((*it).c_str(), "\n") == 0)
		{
			it = tokens.erase(it);
		}
		else if ((*it).find("\n") != string::npos)
		{
			replaceStr((*it), "\n", "", (*it), -1);
			it++;
		}
		else
		{
			it++;
		}
	}

	return 0;
}


double str2num(string s)
{	//字符串转数字，包括整数、小数和科学记数法 
	int i, j, k, negative = 0;
	double n = 0;
	string s1, s2;

	if (s.empty()) return 0;
	if (s[0] == '-') negative = 1; //设置负数标记 
	if (s[0] == '+' || s[0] == '-') s = s.substr(1, s.size());
	//--------------- 
	for (i = 0; i < s.size(); i++)  // 排除不需要的字符 
	{
		if (isNum(s[i]) == -1) return pow(-1.1, 1.1);
	}
	if (s[0] == 'e' || s[0] == '.' || s[s.size() - 1] == 'e' || s[s.size() - 1] == '.')
	{
		return pow(-1.1, 1.1); //排除 e或. 出现在首尾 
	}

	i = -1; j = 0;
	while ((i = s.find('.', ++i)) != s.npos)
	{
		j++;
	}

	if (j > 1)
	{ 
		return pow(-1.1, 1.1);
	} //排除多个小数点 

	i = -1; j = 0;
	while ((i = s.find('e', ++i)) != s.npos)
	{
		j++;
	}

	if (j > 1) return pow(-1.1, 1.1); //排除多个字母e 
	if (s.find('e') == s.npos) //没有e时排除加减
	{
		if (s.find('+') != s.npos || s.find('-') != s.npos) return pow(-1.1, 1.1);
	}

	//---------------
	if ((i = s.find('e')) != s.npos)
	{
		s1 = s.substr(0, i);  // 尾数部分 
		s2 = s.substr(i + 1, s.size());  // 阶码 
		if (s2[0] == '+') s2 = s2.substr(1, s2.size());  // 阶码为正数，去掉+ 
		if (s2.find('.') != s2.npos) return pow(-1.1, 1.1);  // 阶码不准出现小数
		n = str2num(s1)*pow(10.0, str2num(s2));  // 尾数和阶码分别递归调用 
		return negative ? -n : n;
	}

	i = 0; k = 1;
	if ((i = s.find('.')) != s.npos)
	{
		for (j = i + 1; j < s.length(); j++, k++)
		{
			n += isNum(s[j]) / pow(10.0, (double)k);
		}
		n += str2num(s.substr(0, i));  // 整数部分递归调用 
	}
	else
	{
		for (j = 0; j < s.size(); j++)
		{
			n = n * 10 + isNum(s[j]);
		}
	}

	return negative ? -n : n;  // 负数返回-n 
}


const int getDirs(const string& path, vector<string>& dirs)
{
	intptr_t hFile = 0;  // 文件句柄  64位下long 改为 intptr_t
	struct _finddata_t file_info;  // 文件信息 
	string p;
	if ((hFile = _findfirst(p.assign(path).append("/*").c_str(), &file_info)) != -1)  // 文件是否存在
	{
		do
		{
			if ((file_info.attrib & _A_SUBDIR))  // 判断是否为文件夹(目录)
			{
				if (strcmp(file_info.name, ".") != 0 && strcmp(file_info.name, "..") != 0)
				{
					dirs.push_back(p.assign(path).append("/").append(file_info.name));
				}
			}
		} while (_findnext(hFile, &file_info) == 0);
		_findclose(hFile);
	}

	return int(dirs.size());
}


const int getFilesFormat(const string& path, const string& format, vector<string>& files)
{
	intptr_t hFile = 0;  // 文件句柄  64位下long 改为 intptr_t
	struct _finddata_t file_info;  // 文件信息 
	string p;
	if ((hFile = _findfirst(p.assign(path).append("/*" + format).c_str(), &file_info)) != -1)  // 文件存在
	{
		do
		{
			if ((file_info.attrib & _A_SUBDIR))  // 判断是否为文件夹
			{
				if (strcmp(file_info.name, ".") != 0 && strcmp(file_info.name, "..") != 0)  // 文件夹名中不含"."和".."
				{
					files.push_back(p.assign(path).append("/").append(file_info.name));  // 保存文件夹名
					getFilesFormat(p.assign(path).append("/").append(file_info.name), format, files);  // 递归遍历文件夹
				}
			}
			else
			{
				files.push_back(p.assign(path).append("/").append(file_info.name));  // 如果不是文件夹，储存文件名
			}
		} while (_findnext(hFile, &file_info) == 0);
		_findclose(hFile);
	}

	return int(files.size());
}
