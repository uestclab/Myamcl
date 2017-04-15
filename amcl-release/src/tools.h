/* 
 * Name       : tools.h
 * Author     : Xiaoafei
 * Copyright  : 2017 The Prosper Tech
 */

#ifndef __TOOLS_H
#define __TOOLS_H
#include <unistd.h>
#include <string>
#include <string.h>
#include <iostream>
#include <fstream>
#include <stdio.h>  
#include <dirent.h> 

#include <stdlib.h>
#include <fcntl.h>

#include "yaml-cpp/yaml.h"

using namespace std;

int str2char(string s, char c[]);
bool dir_file_exists(string dir, bool mkdir_flag);
string pwd(void);
void run_sys_cmd(string cmd);
vector<string> get_files(string cate_dir);

void WriteMappingInfoToYaml(uint32_t uid,int floor);
void WriteCarStatusToYaml(std::string path, const std::string &filename,
                          uid_t map_uid,float x,float y,float th);
bool ReadCarStatusFromYaml(std::string file_path,
                           uid_t *map_uid,float *x,float *y,float *th,
                           string *map_png,string *map_yaml);

void ReadMapInfoTFromYaml(std::string filename, float *xorigin,float *yorigin,float *resolution);

#endif
