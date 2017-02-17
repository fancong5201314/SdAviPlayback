
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <dirent.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>

#include <net/if.h>
#include <sys/time.h>
#include <sys/socket.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <semaphore.h>

#include "aviapi.h"

#define SD_ERR(format, args...) do{printf("\033[1;31m[SD][%s-%d]\033[m" format, __FILE__, __LINE__, ##args);}while(0)
#define SD_DBG(format, args...) do{printf("\033[1;32m[SD][%s-%d]\033[m" format, __FILE__, __LINE__, ##args);}while(0)
#define SD_INF(format, args...) do{printf("\033[1;35m[SD][%s-%d]\033[m" format, __FILE__, __LINE__, ##args);}while(0)
#define SD_SYS(format, args...) do{printf("\033[1;32m[SD][%s-%d](%s) \033[m" format, __FILE__, __LINE__, strerror(errno), ##args);}while(0)

/* unsigned long类型传入，在32位系统下，时间戳上限2^32=4294967296，即北京时间2106/2/7 14:28:16之后会产生溢出 */
#define ULONG_TO_TIME(ul_time, year, mon, day, hour, min, sec)                    \
    ({                                                                            \
       char tmp[32] = {0};                                                        \
       struct tm cur_local_time;                                                  \
       localtime_r(&ul_time, &cur_local_time);                                    \
       strftime(tmp, sizeof(tmp), "%Y:%m:%d:%H:%M:%S", &cur_local_time);          \
       sscanf(tmp, "%d:%d:%d:%d:%d:%d", &year, &mon, &day, &hour, &min, &sec);    \
    })

/* 字符串时间格式为"year-mon-day hour:min:sec",传出的时间戳为unsigned long类型 */
#define TIME_TO_ULONG(string_time, ul_time)                    \
    ({                                                         \
        struct tm tm;                                          \
        strptime(string_time, "%Y-%m-%d %H:%M:%S", &tm);       \
        ul_time = mktime(&tm);                                 \
    })

/* 定义所需的时间变量 */
#define DECLARE_SHOT_CLOCK() struct timeval ss_start,ss_end,ss_diff;

/* 计时开始.用于性能测试 */  
#define START_SHOT_CLOCK()   do{gettimeofday(&ss_start, NULL);}while(0);

/* 计时结束.用于性能测试 */  
#define STOP_SHOT_CLOCK()    do{gettimeofday(&ss_end, NULL);}while(0);

/* 用于计算STOP_SHOT_CLOCK到START_SHOT_CLOCK阶段所消耗的时间(累加, 微秒) */
#define SPEND_SHOT_CLOCK(ss_total)                                 \
    do{                                                            \
        timersub(&ss_end, &ss_start, &ss_diff);                    \
        ss_total += (ss_diff.tv_sec*1000*1000 + ss_diff.tv_usec); \
    }while(0);

/* 计时结束.合并结束和计算步骤，计算STOP_SHOT_CLOCK到当前阶段所消耗的时间(累加, 微秒) */  
#define STOP_SPEND_SHOT_CLOCK(ss_total)                             \
    do{                                                             \
        gettimeofday(&ss_end, NULL);                                \
        timersub(&ss_end, &ss_start, &ss_diff);                     \
        ss_total += (ss_diff.tv_sec*1000*1000 + ss_diff.tv_usec);  \
    }while(0);

/* 用suffix替换字符串src第二个下划线后面的字符串，放入dest里面 */
#define REPLACE_SECOND_UNDERLINE_NEW_SUFFIX(src, suffix, dest) \
    ({                                                         \
    char tmp[64]={0};                                          \
    char prefix_1[64]={0};                                     \
    char prefix_2[64]={0};                                     \
    sscanf(src, "%[^.].%[^.].%[^.]", prefix_1, prefix_2, tmp); \
    sprintf(dest, "%s_%s_%s", prefix_1, prefix_2, suffix);     \
    })
    
/* 用suffix替换字符串src点后面的字符串，放入dest里面 */
#define REPLACE_AFTER_POINT_TO_SUFFIX(src, suffix, dest)  \
    ({                                                    \
    char tmp[64]={0};                                     \
    char prefix[64]={0};                                  \
    sscanf(src, "%[^.].%[^.]", prefix, tmp);              \
    sprintf(dest, "%s.%s", prefix, suffix);               \
    })

#define MAX_I_FRAME_NUM          (1024)
#define MAX_AVI_NUM              (10000)
#define MAX_FOUND_AVI_NUM        (7200)

#define SD_MOUNT_POINT           "/dat/mmcblk0p1"

#define XOR_INT_64BYTES(p)       (p[0]^p[1]^p[2]^p[3]^p[4]^p[5]^p[6]^p[7]^p[8]^p[9]^p[10]^p[11]^p[12]^p[13]^p[14])

/* 单个AVI文件信息，固定为64个字节 */
typedef struct avi_file_set_s
{
    unsigned int  is_available;     /* AVI文件是否有效       */
    unsigned char file_name[32];    /* AVI文件的文件名字     */
    unsigned int  start_time;       /* AVI文件开始录制的时间 */
    unsigned int  stop_time;        /* AVI文件结束录制的时间 */
    unsigned int  record_type;      /* AVI文件的文件类型     */
    unsigned int  resolve[3];       /* 保留字段              */
    unsigned int  struct_xor_crc;   /* 结构体按位异或校验结果*/
}__attribute__((packed)) Avi_file_info, *pAvi_file_info;

/* SD卡总索引文件，固定为64个字节 */
typedef struct global_head_s
{
    int head_SEI;                  /* 固定为0X7765767a 校验使用  */
    int head_verify;               /* 固定为0X1002feea 校验使用  */
    int write_file_index;          /* 正在写哪一个文件(文件编号) */
    int storage_capacity;          /* 分区总空间(MB)             */
    int storage_used_capacity;     /* 已经用了多少(MB)           */
    int video_num;                 /* 能够容纳多少个录像         */
    int picture_num;               /* 能够容纳多少张图片         */
    int into_circle_record;        /* 是否进入循环录像了         */
    int resolve[7];                /* 保留字段                   */    
    int struct_xor_crc;            /* 结构体按位异或校验结果     */
}__attribute__((packed)) global_head_t;

/* 每个AVI的I帧信息，固定为16个字节 */
typedef struct tagIFrameInfo
{
    unsigned int CurIFrameTime;  /* 当前时间                        */
    unsigned int IFrameNum;      /* 当前I帧编号在总视频帧里面的编号 */
    unsigned int resolve[2];     /* 保留字段                        */
}__attribute__((packed))IFrameInfo, *pIFrameInfo;

static global_head_t g_global_head;                 // 全局索引头(64Bytes)

static Avi_file_info g_avi_file_set[MAX_AVI_NUM];   // 从全局索引获取到的AVI文件信息集合(625KBytes)

static int g_curr_avi_num = 0;                      // 当前全局索引里面的有效AVI个数
static int g_curr_avi_index = 0;                    // 当前操作的AVI的g_avi_file_set下标

static int g_seek_seg_start_index = 0;              // 当前seek片段的g_avi_file_set的起始下标
static int g_seek_seg_stop_index  = 0;              // 当前seek片段的g_avi_file_set的结束下标

/*@breif BinareySearchEx 二分法查找递增表中最接近的数
 *@param [IN] data 欲查找的递增数组表
 *@param [IN] data_count 该数组的元素个数
 *@param [IN] key 欲查找的数
 *@return 成功返回数组下标
 *@       失败返回-1
 *@note   支持重复数据, 返回最近最大的数的下标减一
 */
inline int BinareySearchEx(int *data, int data_count, int key)
{
    if(!data  || data_count < 1)
    {
        SD_ERR("NULL POINTER EXCEPTION!\n");
        return -1;
    }

    if(data_count == 1)
    {         
        return data_count-1;
    }  

    if(key <= data[0])   
    {  
        SD_DBG("warnning: key = %d, data[0] = %d\n", key, data[0]);            
        return 0;  
    }     
    else if(key >= data[data_count-1])    
    {       
        SD_DBG("warnning: key = %d, data[%d] = %d\n", key, data_count -1, data[data_count-1]); 
        return data_count-1;    
    }  

    int *start  = data;
    int *end    = data + (data_count - 1);
    int *middle = NULL;

    if(*end <= key)
    {
        return data_count;
    }

    while(start < end)
    {
        middle = start + ((end-start) >> 1);
        if(*middle > key) 
        {
            end = middle;
        }
        else
        {
            start = ++middle;
        }
    }

    return start-data-1; /* 返回不超过的最大下标 */
}

static int cmp_avi_start_time(const void *a, const void *b)
{
    Avi_file_info *aa = (Avi_file_info *)a;
    Avi_file_info *bb = (Avi_file_info *)b;
    return ((aa->start_time)>(bb->start_time)?1:-1);
}

static int com_avi_i_frame_time(const void *a, const void *b)
{
    IFrameInfo *aa = (IFrameInfo *)a;
    IFrameInfo *bb = (IFrameInfo *)b;
    return ((aa->CurIFrameTime)>(bb->CurIFrameTime)?1:-1);
}

/*@breif GetGlobalHeadInfoFromGlobalFile 从全局文件里面获取信息头和AVI信息放在全局结构体里面
 *@return 成功返回0
 *@       失败返回-1
 */
int GetGlobalHeadInfoFromGlobalFile(void)
{
    char global_file[64] = {0};
    sprintf(global_file, "%s/global_index.db", SD_MOUNT_POINT);

    int fd = open(global_file, O_RDWR, S_IRWXU);
    if (-1 == fd)
    {
        SD_SYS("open %s fail!\n", global_file);
        return -1;
    }
    
    /* 定位到文件头，读取64字节的全局索引头 */
    lseek(fd, 0, SEEK_SET);
    int read_size = read(fd, &g_global_head, sizeof(g_global_head));
    if (sizeof(global_head_t) != read_size)
    {
        SD_ERR("read global head info fail!\n");
        close(fd);
        return -1;
    }

    int *p = (int *)&g_global_head;

    /* 校验结构体的正确性 */
    if (g_global_head.head_SEI != 0x7765767a 
     || g_global_head.head_verify != 0x1002feea 
     || g_global_head.struct_xor_crc != XOR_INT_64BYTES(p))
    {
        SD_ERR("global head info crc fail!\n");
        close(fd);
        return -1;
    }

    int i = 0;
    
    /* 从全局文件里面获取每个avi的信息放在结构体里面 */
    while(1)
    {
        if (0 == read(fd, &g_avi_file_set[i], sizeof(Avi_file_info)))
        {
            break;
        }
        i++;
    }

    g_curr_avi_num = i;

    qsort(g_avi_file_set, g_curr_avi_num, sizeof(Avi_file_info), cmp_avi_start_time);

    close(fd);
    fd = -1;
    
    return 0;
}

/*@breif GetIFrameListFromAviFile 从avi文件里面获取I帧信息表放在句柄里面
 *@param [IN]  avi_file 欲操作的AVI文件
 *@param [OUT] i_frame_handle 从avi_file里面获取到的I帧信息句柄
 *@return 成功返回0
 *@       失败返回-1
 */
int GetIFrameListFromAviFile(const char *avi_file, void *i_frame_handle)
{
    if (!avi_file || !i_frame_handle)
    {
        SD_ERR("NULL POINTER EXCEPTION!\n");
        return -1;
    }

    IFrameInfo *i_frame_list = (IFrameInfo *)i_frame_handle;

    char data[4] = {0};
    int fd = open(avi_file, O_RDWR, S_IRWXU);
    if (-1 == fd)
    {
        SD_SYS("open %s fail!\n", avi_file);
        return -1;
    }

    /* 读取AVI头，从里面获取AVI数据大小(不包含I帧信息的纯AVI数据大小) */
    lseek(fd, 4, SEEK_SET);
    if (4 != read(fd, data, 4))
    {
        SD_ERR("read avi_size fail!\n");
        close(fd);
        return -1;
    }

    int avi_size = ((data[0]&0xff)|((data[1]&0xff)<<8)|((data[2]&0xff)<<16)|((data[3]&0xff)<<24)) + 8;
    
    /* 定义文件指针到纯净AVI数据尾部 */
    lseek(fd, avi_size, SEEK_SET);

    /* 直接读取16KB的INFO信息大小 */
    read(fd, i_frame_list, MAX_I_FRAME_NUM*sizeof(IFrameInfo));

    close(fd);
    fd = -1;

    qsort(i_frame_list, MAX_I_FRAME_NUM, sizeof(IFrameInfo), com_avi_i_frame_time);

    return 0;    
}

/*@breif GetAviFileList 从给定的时间段里面获取符合该时间段的avi文件列表
 *@param [IN]  start_time 欲查找的开始时间戳
 *@param [IN]  stop_time  欲查找的结束时间戳
 *@param [OUT] file_list  获取到的文件指针数组
 *@param [OUT] avi_count  获取到的AVI文件个数
 *@return 成功返回0
 *@       失败返回-1
 *@note   查询的时间段不能超过一天即24小时
 */
int GetTimeSegmentArray(int start_time, int stop_time, void **file_list, int *avi_file_count)
{
    if (!file_list || !avi_file_count)
    {
       SD_ERR("NULL POINTER EXCEPTION!\n");
       return -1;
    }

    if (stop_time < start_time)
    {
        SD_ERR("Seek time format error!\n");
        return -1;
    }

    /* 搜索时间限定不能超过24H */
    if (stop_time - start_time > 24*60*60)
    {
        SD_ERR("Find time segment too long! not more than 24h!\n");
        return -1;
    }

    start_time = start_time<g_avi_file_set[0].start_time?g_avi_file_set[0].start_time:start_time;
    stop_time  = stop_time>g_avi_file_set[g_curr_avi_num-1].stop_time?g_avi_file_set[g_curr_avi_num-1].stop_time:stop_time;

    SD_DBG("Global index file avi time from %d to %d\n", g_avi_file_set[0].start_time, g_avi_file_set[g_curr_avi_num-1].stop_time);

    int i = 0, j = 0;
    int m = 0, n = 0;
    int curr_avi_times[MAX_FOUND_AVI_NUM] = {0};
    char **avi_file_list = (char **)file_list;

    for (i=0; i<g_curr_avi_num; i++)
    {
        curr_avi_times[i] = g_avi_file_set[i].start_time;
    }

    m = BinareySearchEx(curr_avi_times, g_curr_avi_num, start_time);
    n = BinareySearchEx(curr_avi_times, g_curr_avi_num, stop_time);

    for (i=m, j=0; i<=n; i++, j++)
    {
        strcpy(avi_file_list[j], g_avi_file_set[i].file_name);
    }
    
    *avi_file_count = j;
    g_seek_seg_start_index = m;
    g_seek_seg_stop_index  = n;

    return 0;
}

/*@breif SeekTimePointFromAviFileList 从给出的文件链表结构体里面获取对应的AVI文件进而获取对应的I帧帧号
 *@param [IN] seek_time 欲查找的开始时间戳
 *@param [OUT] get_avi_name   获取到的AVI文件
 *@param [OUT] get_iframe_num 获取到的I帧对应的视频帧的帧号
 *@return 成功返回0
 *@       失败返回-1
 *@note   需要先调用GetTimeSegmentArray获取对应的文件列表，然后从里面获取文件以及对应I帧帧号
 */
int SeekTimePointFromAviFileList(int seek_time, char *get_avi_name, int *get_iframe_num)
{
    if (!get_iframe_num || !get_avi_name)
    {
       SD_ERR("NULL POINTER EXCEPTION!\n");
       return -1;
    }

    /* 定位的开始下标不能超过结束下标 */
    if (g_seek_seg_start_index > g_seek_seg_stop_index)
    {
        SD_ERR("Segment index [%d > %d] fail!\n", g_seek_seg_start_index, g_seek_seg_stop_index);
        return -1;
    }

    /* 定位的结束下标不能数据越界 */
    if (g_seek_seg_stop_index > g_curr_avi_num)
    {
        SD_ERR("Segment stop index %d out of rang %d!\n", g_seek_seg_stop_index, g_curr_avi_num);
        return -1;
    }

    /* 判断定位的区域是否有录像 */
    if (seek_time < g_avi_file_set[g_seek_seg_start_index].start_time || seek_time > g_avi_file_set[g_seek_seg_stop_index].stop_time)
    {
        SD_ERR("seek time %d is out of [%d, %d]\n", seek_time, g_avi_file_set[g_seek_seg_start_index].start_time, g_avi_file_set[g_seek_seg_stop_index].stop_time);
        return -1;
    }

    int i = 0, j = 0, ret = -1;
    int avi_start_time_list[MAX_FOUND_AVI_NUM] = {0};
    IFrameInfo iframe_list[MAX_I_FRAME_NUM];

    for (i=g_seek_seg_start_index, j=0; i<=g_seek_seg_stop_index; i++, j++)
    {
        avi_start_time_list[j] = g_avi_file_set[i].start_time;
    }

    /* 二分法查找获取到最接近的文件名 */
    i = BinareySearchEx(avi_start_time_list, j, seek_time);

    g_curr_avi_index = i + g_seek_seg_start_index;
    
    strncpy(get_avi_name, g_avi_file_set[g_curr_avi_index].file_name, sizeof(g_avi_file_set[g_curr_avi_index].file_name));

    /* 从当前AVI文件里面获取最接近的I帧帧号 */
    char avi_file[64] = {0};
    sprintf(avi_file, "%s/video/%s", SD_MOUNT_POINT, g_avi_file_set[g_curr_avi_index].file_name);

    ret = GetIFrameListFromAviFile(avi_file, iframe_list);
    if (ret)
    {
        SD_ERR("Fail to called GetIFrameListFromAviFile\n");
        return -1;
    }

    int iframe_time_list[MAX_I_FRAME_NUM] = {0};
    
    for (i=0; i<MAX_I_FRAME_NUM; i++)
    {
        iframe_time_list[i] = iframe_list[i].CurIFrameTime;
    }

    j = BinareySearchEx(iframe_time_list, i, seek_time);

    int y, m, d, h, min, s;
    ULONG_TO_TIME(iframe_list[j].CurIFrameTime, y, m, d, h, min, s);
    SD_DBG("found iframe time %d ---> %04d-%02d-%02d %02d:%02d:%02d\n", iframe_list[j].CurIFrameTime, y, m, d, h, min, s);

    *get_iframe_num = iframe_list[j].IFrameNum;

    return 0;
}

/*@breif SeekTimePointFromGlobalIndexFile 从全局索引文件里面获取对应的AVI文件进而获取对应的I帧帧号
 *@param [IN] seek_time 欲查找的开始时间戳
 *@param [OUT] get_avi_name 获取到的AVI文件
 *@param [OUT] get_iframe_num 获取到的I帧对应的视频帧的帧号
 *@return 成功返回0
 *@       失败返回-1
 */
int SeekTimePointFromGlobalIndexFile(int seek_time, char *get_avi_name, int *get_iframe_num)
{
    if (!get_iframe_num || *get_avi_name)
    {
       SD_ERR("NULL POINTER EXCEPTION!\n");
       return -1;
    }

    /* 判断定位的区域是否有录像 */
    if (seek_time < g_avi_file_set[0].start_time || seek_time > g_avi_file_set[g_curr_avi_num-1].stop_time)
    {
        SD_ERR("seek time %d is out of [%d, %d]\n", seek_time, g_avi_file_set[0].start_time, g_avi_file_set[g_curr_avi_num-1].stop_time);
        return -1;
    }

    /* 在全局AVI信息结构体里面寻找最接近该定位点的AVI文件，获取其AVI文件名 */
    int i = 0;
    int ret = 0;
    int avi_file_list[MAX_AVI_NUM] = {0};
    int iframe_num_list[MAX_I_FRAME_NUM] = {0};
    char avi_file[64] = {0};
    IFrameInfo iframe_list[MAX_I_FRAME_NUM];

    for (i=0; i<g_curr_avi_num; i++)
    {
        avi_file_list[i] = g_avi_file_set[i].start_time;
    }

    i = BinareySearchEx(avi_file_list, g_curr_avi_num, seek_time);

    strncpy(get_avi_name, g_avi_file_set[i].file_name, sizeof(g_avi_file_set[i].file_name));

    sprintf(avi_file, "%s/video/%s", SD_MOUNT_POINT, g_avi_file_set[i].file_name);

    ret = GetIFrameListFromAviFile(avi_file, iframe_list);
    if (ret)
    {
        SD_ERR("Fail to called GetIFrameListFromAviFile\n");
        return -1;
    }

    for (i=0; i<MAX_I_FRAME_NUM; i++)
    {
        iframe_num_list[i] = iframe_list[i].CurIFrameTime;
    }

    i = BinareySearchEx(iframe_num_list, MAX_I_FRAME_NUM, seek_time);

    *get_iframe_num = iframe_list[i].IFrameNum;

    return 0;
}

int g_curr_seek_iframe_num = 0;

sem_t g_sem_start_playback;

void *PlaybackThread(void *lparam)
{
    int i = 0;
    int ret = 0;
    int keyframe = 0;
    int framesize = 0;
    int frametype = 0; // 当前帧的类型 0:视频帧  1:音频帧

    char ops_avi_file[64] = {0};
    char FrameData[512*1024] = {0};
        
    while(1)
    {
        sem_wait(&g_sem_start_playback);

        int y=0,mon=0,d=0,h=0,min=0,s=0;
        unsigned int curr_sec = 0;
        unsigned char out_video_file[64] = {0};
        unsigned char out_audio_file[64] = {0};
        
        curr_sec = time((time_t*)NULL);
        ULONG_TO_TIME(curr_sec, y, mon, d, h, min, s);
        
        sprintf(out_video_file, "%s/%04d%02d%02d%02d%02d%02d.264", SD_MOUNT_POINT, y, mon, d, h, min, s);
        sprintf(out_audio_file, "%s/%04d%02d%02d%02d%02d%02d.pcm", SD_MOUNT_POINT, y, mon, d, h, min, s);
        
        FILE *video_fp = fopen(out_video_file, "w+");
        FILE *audio_fp = fopen(out_audio_file, "w+");

        for (i=g_curr_avi_index; i<=g_seek_seg_stop_index; i++)
        {
            sprintf(ops_avi_file, "%s/video/%s", SD_MOUNT_POINT, g_avi_file_set[i].file_name);
            InitAviActionMode(ops_avi_file, 1);
            SetAviIndexPosition(g_curr_seek_iframe_num);
            for (;;)
            {
                ret = ReadAviFrame(FrameData, &framesize, &frametype, &keyframe);
                if (ret == 0)
                {
                    if (frametype)
                    {
                        fwrite(FrameData, framesize, 1, video_fp);
                    }
                    else
                    {

                        fwrite(FrameData, framesize, 1, audio_fp);
                    }
                }
                else if (1 == ret)
                {
                    SD_DBG("Read %s Over!\n", g_avi_file_set[i].file_name);
                    StopAviAction(1);
                    g_curr_seek_iframe_num = 0;
                    break;
                }
            }
        }
        fclose(video_fp);
        fclose(audio_fp);
        SD_DBG("All avi file read over!\n");
    }

    return NULL;
}

int main(int argc, const char *argv[])
{
    if (argc != 4)
    {
        SD_ERR("Usage: %s segment_start_time_stamp segment_stop_time_stamp seek_time_stamp\n", argv[0]);
        return -1;
    }

    int y, m, d, h, min, s;
    unsigned long timestamp = 0;

    timestamp = atol(argv[1]);
    ULONG_TO_TIME(timestamp, y, m, d, h, min, s);
    SD_INF("start_time: %s ---> %04d-%02d-%02d %02d:%02d:%02d\n", argv[1], y, m, d, h, min, s);

    timestamp = atol(argv[2]);
    ULONG_TO_TIME(timestamp, y, m, d, h, min, s);
    SD_INF("stop_time:  %s ---> %04d-%02d-%02d %02d:%02d:%02d\n", argv[2], y, m, d, h, min, s);

    timestamp = atol(argv[3]);
    ULONG_TO_TIME(timestamp, y, m, d, h, min, s);
    SD_INF("seek_time:  %s ---> %04d-%02d-%02d %02d:%02d:%02d\n", argv[3], y, m, d, h, min, s);

    int ret = 0;
    int get_iframe_num = 0;    
    pthread_t paly_back_id = -1;
    
    unsigned int spent_time_ms = 0;

    DECLARE_SHOT_CLOCK();
    
    sem_init(&g_sem_start_playback, 0, 0);

    pthread_create(&paly_back_id, NULL, PlaybackThread, NULL);

    START_SHOT_CLOCK();

    ret = GetGlobalHeadInfoFromGlobalFile();
    if (ret)
    {
        SD_ERR("Fail to called GetGlobalHeadInfoFromGlobalFile\n");
        return -1;
    }

    int i = 0;
    int avi_count = 0;
    char *file_list[MAX_FOUND_AVI_NUM] = {0};    
    char avi_file[32] = {0};

    for(i=0; i<MAX_FOUND_AVI_NUM; i++)
    {
        file_list[i] = (char *)malloc(32*sizeof(char));
    }

    ret = GetTimeSegmentArray(atoi(argv[1]), atoi(argv[2]), (void**)file_list, &avi_count);
    if (ret)
    {
        SD_ERR("Fail to called GetTimeSegmentArray\n");
        return -1;
    }

//  for (i=0; i<avi_count; i++)
    {
//      SD_INF("found NO.%d avi is %s\n", i, file_list[i]);
    }

    for (i=0; i<MAX_FOUND_AVI_NUM; i++)
    {
        free(file_list[i]);
    }

    ret = SeekTimePointFromAviFileList(atoi(argv[3]), avi_file, &get_iframe_num);
    if (ret)
    {
        SD_ERR("Fail to called SeekTimePointFromAviFileList\n");
        return -1;
    }
    
    STOP_SPEND_SHOT_CLOCK(spent_time_ms);

    SD_INF("Found avi file is %s\n", avi_file);

    SD_INF("Found iframe num is %d\n", get_iframe_num);

    SD_INF("Found info spent %d ms\n", spent_time_ms/1000);

    g_curr_seek_iframe_num = get_iframe_num;

    sem_post(&g_sem_start_playback);
    
    SD_INF("Playbacking ......\n");
    
    while(1)
    {
        sleep(10);
    }

#if 0

    START_SHOT_CLOCK();
    
    ret = GetGlobalHeadInfoFromGlobalFile();
    if (ret)
    {
        SD_ERR("Fail to called GetGlobalHeadInfoFromGlobalFile\n");
        return -1;
    }
    
    avi_file[0] = '\0';
    ret = SeekTimePointFromGlobalIndexFile(atoi(argv[3]), avi_file, &get_iframe_num);
    if (ret)
    {
        SD_ERR("Fail to called SeekTimePointFromGlobalIndexFile\n");
        return -1;
    }
    
    STOP_SPEND_SHOT_CLOCK(spent_time_ms);

    SD_INF("Found avi file is %s\n", avi_file);
    
    SD_INF("Found iframe num from global file is %d\n", get_iframe_num);

    SD_INF("Found info from global file spent %d ms\n", spent_time_ms/1000);

#endif

    return 0;
}
