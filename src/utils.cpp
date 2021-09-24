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
	ret = string(src_str);  // string�Ŀ�������

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
	// �ַ�������Ԥ����
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
{	//�ַ���ת���֣�����������С���Ϳ�ѧ������ 
	int i, j, k, negative = 0;
	double n = 0;
	string s1, s2;

	if (s.empty()) return 0;
	if (s[0] == '-') negative = 1; //���ø������ 
	if (s[0] == '+' || s[0] == '-') s = s.substr(1, s.size());
	//--------------- 
	for (i = 0; i < s.size(); i++)  // �ų�����Ҫ���ַ� 
	{
		if (isNum(s[i]) == -1) return pow(-1.1, 1.1);
	}
	if (s[0] == 'e' || s[0] == '.' || s[s.size() - 1] == 'e' || s[s.size() - 1] == '.')
	{
		return pow(-1.1, 1.1); //�ų� e��. ��������β 
	}

	i = -1; j = 0;
	while ((i = s.find('.', ++i)) != s.npos)
	{
		j++;
	}

	if (j > 1)
	{ 
		return pow(-1.1, 1.1);
	} //�ų����С���� 

	i = -1; j = 0;
	while ((i = s.find('e', ++i)) != s.npos)
	{
		j++;
	}

	if (j > 1) return pow(-1.1, 1.1); //�ų������ĸe 
	if (s.find('e') == s.npos) //û��eʱ�ų��Ӽ�
	{
		if (s.find('+') != s.npos || s.find('-') != s.npos) return pow(-1.1, 1.1);
	}

	//---------------
	if ((i = s.find('e')) != s.npos)
	{
		s1 = s.substr(0, i);  // β������ 
		s2 = s.substr(i + 1, s.size());  // ���� 
		if (s2[0] == '+') s2 = s2.substr(1, s2.size());  // ����Ϊ������ȥ��+ 
		if (s2.find('.') != s2.npos) return pow(-1.1, 1.1);  // ���벻׼����С��
		n = str2num(s1)*pow(10.0, str2num(s2));  // β���ͽ���ֱ�ݹ���� 
		return negative ? -n : n;
	}

	i = 0; k = 1;
	if ((i = s.find('.')) != s.npos)
	{
		for (j = i + 1; j < s.length(); j++, k++)
		{
			n += isNum(s[j]) / pow(10.0, (double)k);
		}
		n += str2num(s.substr(0, i));  // �������ֵݹ���� 
	}
	else
	{
		for (j = 0; j < s.size(); j++)
		{
			n = n * 10 + isNum(s[j]);
		}
	}

	return negative ? -n : n;  // ��������-n 
}


const int getDirs(const string& path, vector<string>& dirs)
{
	intptr_t hFile = 0;  // �ļ����  64λ��long ��Ϊ intptr_t
	struct _finddata_t file_info;  // �ļ���Ϣ 
	string p;
	if ((hFile = _findfirst(p.assign(path).append("/*").c_str(), &file_info)) != -1)  // �ļ��Ƿ����
	{
		do
		{
			if ((file_info.attrib & _A_SUBDIR))  // �ж��Ƿ�Ϊ�ļ���(Ŀ¼)
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
	intptr_t hFile = 0;  // �ļ����  64λ��long ��Ϊ intptr_t
	struct _finddata_t file_info;  // �ļ���Ϣ 
	string p;
	if ((hFile = _findfirst(p.assign(path).append("/*" + format).c_str(), &file_info)) != -1)  // �ļ�����
	{
		do
		{
			if ((file_info.attrib & _A_SUBDIR))  // �ж��Ƿ�Ϊ�ļ���
			{
				if (strcmp(file_info.name, ".") != 0 && strcmp(file_info.name, "..") != 0)  // �ļ������в���"."��".."
				{
					files.push_back(p.assign(path).append("/").append(file_info.name));  // �����ļ�����
					getFilesFormat(p.assign(path).append("/").append(file_info.name), format, files);  // �ݹ�����ļ���
				}
			}
			else
			{
				files.push_back(p.assign(path).append("/").append(file_info.name));  // ��������ļ��У������ļ���
			}
		} while (_findnext(hFile, &file_info) == 0);
		_findclose(hFile);
	}

	return int(files.size());
}
