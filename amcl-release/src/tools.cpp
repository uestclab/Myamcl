/* 
 * Name       : tools.cpp
 * Author     : Xiaoafei
 * Copyright  : 2017 The Prosper Tech
 *
 * brief      : 常用工具函数
 */

#include "tools.h"

int str2char(string s, char c[])
{
    size_t l = s.length();
    int i;
    for (i = 0; i < l; i++)
        c[i] = s[i];
    c[i] = '\0';
    return i;
}

/*运行系统命令*/
void run_sys_cmd(string cmd)
{
    if (system(cmd.c_str()))
    {
    }

    cout << "run--  " << cmd << " --OK!" << endl;
}

/*创建文件夹*/
bool dir_file_exists(string dir, bool mkdir_flag)
{
    char des_dir[255];
    str2char(dir, des_dir);                   // 将string 写入到字符数组中
    int state = access(des_dir, R_OK | W_OK); // 头文件 #include <unistd.h>
    if (state == 0)
    {
        //cout<<dir<<" exist"<<endl;
        return true;
    }
    else if (mkdir_flag)
    {
        dir = "mkdir " + dir;
        str2char(dir, des_dir);
        //cout<<des_dir<<endl;
        if (system(des_dir))
        {
        }
        return true;
    }
    else
    {
        return false;
    }
}

/*输出程序当前路径*/
string pwd(void)
{
    char *buffer;
    if ((buffer = getcwd(NULL, 0)) == NULL)
    {
        cout << "get path error" << endl;

        return NULL;
    }
    else
    {
        string path = buffer;
        free(buffer);

        return path;
    }
}

vector<string> get_files(string cate_dir)
{
    vector<string> files; //存放文件名

    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir = opendir(cate_dir.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) ///current dir OR parrent dir
            continue;
        else if (ptr->d_type == 8) ///file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            files.push_back(ptr->d_name);
        else if (ptr->d_type == 10) ///link file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            continue;
        else if (ptr->d_type == 4)
        { ///dir
            files.push_back(ptr->d_name);
            /* 
                memset(base,'\0',sizeof(base)); 
                strcpy(base,basePath); 
                strcat(base,"/"); 
                strcat(base,ptr->d_nSame); 
                readFileList(base); 
            */
        }
    }
    closedir(dir);

    //排序，按从小到大排序
    sort(files.begin(), files.end());
    return files;
}

/*写入建图相关参数  cartograpgher 调用*/
void WriteMappingInfoToYaml(uint32_t uid, int floor)
{
    std::ofstream yaml_file("/tmp/mapping.yaml", std::ios::out | std::ios::binary);
    {
        cout << "void WriteMappingInfoToYaml..." << endl;

        YAML::Emitter out(yaml_file);
        out << YAML::BeginMap;
        out << YAML::Key << "uid" << YAML::Value << uid;
        out << YAML::Key << "floor" << YAML::Value << floor;
        out << YAML::EndMap;
    }

    yaml_file.close();
}

template <typename T>
void operator>>(const YAML::Node &node, T &i)
{
    i = node.as<T>();
}

/*存储底盘当前状态*/
void WriteCarStatusToYaml(std::string path, const std::string &filename,
                          uid_t map_uid, float x, float y, float th)
{
    dir_file_exists(path, true);

    std::ofstream yaml_file(path + filename, std::ios::out | std::ios::binary);
    {
        YAML::Emitter out(yaml_file);
        out << YAML::BeginMap;
        // TODO(whess): Use basename only?
        out << YAML::Key << "map_uid" << YAML::Value << map_uid;
        out << YAML::Key << "x" << YAML::Value << x;
        out << YAML::Key << "y" << YAML::Value << y;
        out << YAML::Key << "th" << YAML::Value << th;
        out << YAML::Key << "map_png" << YAML::Value << (path + "map_" + to_string(map_uid) + ".png");
        out << YAML::Key << "map_yaml" << YAML::Value << (path + "map_" + to_string(map_uid) + ".yaml");
        out << YAML::EndMap;
    }

    yaml_file.close();
}

/*开机读取状态*/
bool ReadCarStatusFromYaml(std::string file_path,
                           uid_t *map_uid, float *x, float *y, float *th,
                           string *map_png, string *map_yaml)
{

    if (!dir_file_exists(file_path, false))
        return false;

    std::ifstream yaml_file(file_path);
    {
        YAML::Node doc = YAML::Load(yaml_file);

        doc["map_uid"] >> *map_uid;
        doc["x"] >> *x;
        doc["y"] >> *y;
        doc["th"] >> *th;
        doc["map_png"] >> *map_png;
        doc["map_yaml"] >> *map_yaml;
    }

    yaml_file.close();

    return true;
}

void ReadMapInfoTFromYaml(std::string filename, float *xorigin, float *yorigin, float *resolution)
{
    std::ifstream yaml_file(filename);
    {
        YAML::Node doc = YAML::Load(yaml_file);
        // doc["uid"] >> msg.uid;
        // doc["xmin"] >> msg.xmin;
        // doc["ymin"] >> msg.ymin;
        // doc["xmax"] >> msg.xmax;
        // doc["ymax"] >> msg.ymax;
        doc["xorigin"] >> *xorigin;
        doc["yorigin"] >> *yorigin;
        doc["resolution"] >> *resolution;
        // doc["floor"] >> msg.floor;
    }

    yaml_file.close();
}
