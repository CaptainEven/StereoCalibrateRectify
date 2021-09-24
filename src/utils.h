#ifndef UTILS
#define UTILS

#include <io.h>
#include <string>
#include <vector>

#define isNum(c) (isdigit(c)?c-48:(c=='e'?10:(c=='.'?11:(c=='-'?12:(c=='+'?13:-1)))))


using namespace std;

// string operations
void splitStr(const string& s, vector<string>& tokens, const char& delim);

void replaceStr(const string& src_str, const string &old_str, const string& new_str, string& ret, int count);

int filterStr(vector<string>& tokens);

double str2num(string s);

// file operations
const int getDirs(const string& path, vector<string>& dirs);

const int getFilesFormat(const string& path, const string& format, vector<string>& files);

#endif

