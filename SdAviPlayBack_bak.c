
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

#define SD_ERR(format, args...) do{printf("\033[1;31m[SD][%s-%d]\033[m" format, __FILE__, __LINE__, ##args);}while(0)
#define SD_DBG(format, args...) do{printf("\033[1;32m[SD][%s-%d]\033[m" format, __FILE__, __LINE__, ##args);}while(0)
#define SD_INF(format, args...) do{printf("\033[1;35m[SD][%s-%d]\033[m" format, __FILE__, __LINE__, ##args);}while(0)
#define SD_SYS(format, args...) do{printf("\033[1;32m[SD][%s-%d](%s) \033[m" format, __FILE__, __LINE__, strerror(errno), ##args);}while(0)

/* unsigned long���ʹ��룬��32λϵͳ�£�ʱ�������2^32=4294967296��������ʱ��2106/2/7 14:28:16֮��������� */
#define ULONG_TO_TIME(ul_time, year, mon, day, hour, min, sec)                    \
    ({                                                                            \
       char tmp[32] = {0};                                                        \
       struct tm cur_local_time;                                                  \
       localtime_r(&ul_time, &cur_local_time);                                    \
       strftime(tmp, sizeof(tmp), "%Y:%m:%d:%H:%M:%S", &cur_local_time);          \
       sscanf(tmp, "%d:%d:%d:%d:%d:%d", &year, &mon, &day, &hour, &min, &sec);    \
    })

/* �ַ���ʱ���ʽΪ"year-mon-day hour:min:sec",������ʱ���Ϊunsigned long���� */
#define TIME_TO_ULONG(string_time, ul_time)                    \
    ({                                                         \
        struct tm tm;                                          \
        strptime(string_time, "%Y-%m-%d %H:%M:%S", &tm);       \
        ul_time = mktime(&tm);                                 \
    })

/* ���������ʱ����� */
#define DECLARE_SHOT_CLOCK() struct timeval ss_start,ss_end,ss_diff;

/* ��ʱ��ʼ.�������ܲ��� */  
#define START_SHOT_CLOCK()   do{gettimeofday(&ss_start, NULL);}while(0);

/* ��ʱ����.�������ܲ��� */  
#define STOP_SHOT_CLOCK()    do{gettimeofday(&ss_end, NULL);}while(0);

/* ���ڼ���STOP_SHOT_CLOCK��START_SHOT_CLOCK�׶������ĵ�ʱ��(�ۼ�, ΢��) */
#define SPEND_SHOT_CLOCK(ss_total)                                 \
    do{                                                            \
        timersub(&ss_end, &ss_start, &ss_diff);                    \
        ss_total += (ss_diff.tv_sec*1000*1000 + ss_diff.tv_usec); \
    }while(0);

/* ��ʱ����.�ϲ������ͼ��㲽�裬����STOP_SHOT_CLOCK����ǰ�׶������ĵ�ʱ��(�ۼ�, ΢��) */  
#define STOP_SPEND_SHOT_CLOCK(ss_total)                             \
    do{                                                             \
        gettimeofday(&ss_end, NULL);                                \
        timersub(&ss_end, &ss_start, &ss_diff);                     \
        ss_total += (ss_diff.tv_sec*1000*1000 + ss_diff.tv_usec);  \
    }while(0);

/* ��suffix�滻�ַ���src�ڶ����»��ߺ�����ַ���������dest���� */
#define REPLACE_SECOND_UNDERLINE_NEW_SUFFIX(src, suffix, dest) \
    ({                                                         \
    char tmp[64]={0};                                          \
    char prefix_1[64]={0};                                     \
    char prefix_2[64]={0};                                     \
    sscanf(src, "%[^.].%[^.].%[^.]", prefix_1, prefix_2, tmp); \
    sprintf(dest, "%s_%s_%s", prefix_1, prefix_2, suffix);     \
    })
    
/* ��suffix�滻�ַ���src�������ַ���������dest���� */
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

/* ����AVI�ļ���Ϣ���̶�Ϊ64���ֽ� */
typedef struct avi_file_set_s
{
    unsigned int  is_available;     /* AVI�ļ��Ƿ���Ч       */
    unsigned char file_name[32];    /* AVI�ļ����ļ�����     */
    unsigned int  start_time;       /* AVI�ļ���ʼ¼�Ƶ�ʱ�� */
    unsigned int  stop_time;        /* AVI�ļ�����¼�Ƶ�ʱ�� */
    unsigned int  record_type;      /* AVI�ļ����ļ�����     */
    unsigned int  resolve[3];       /* �����ֶ�              */
    unsigned int  struct_xor_crc;   /* �ṹ�尴λ���У����*/
}__attribute__((packed)) Avi_file_info, *pAvi_file_info;

/* SD���������ļ����̶�Ϊ64���ֽ� */
typedef struct global_head_s
{
    int head_SEI;                  /* �̶�Ϊ0X7765767a У��ʹ��  */
    int head_verify;               /* �̶�Ϊ0X1002feea У��ʹ��  */
    int write_file_index;          /* ����д��һ���ļ�(�ļ����) */
    int storage_capacity;          /* �����ܿռ�(MB)             */
    int storage_used_capacity;     /* �Ѿ����˶���(MB)           */
    int video_num;                 /* �ܹ����ɶ��ٸ�¼��         */
    int picture_num;               /* �ܹ����ɶ�����ͼƬ         */
    int into_circle_record;        /* �Ƿ����ѭ��¼����         */
    int resolve[7];                /* �����ֶ�                   */    
    int struct_xor_crc;            /* �ṹ�尴λ���У����     */
}__attribute__((packed)) global_head_t;

/* ÿ��AVI��I֡��Ϣ���̶�Ϊ16���ֽ� */
typedef struct tagIFrameInfo
{
    unsigned int CurIFrameTime;  /* ��ǰʱ��                        */
    unsigned int IFrameNum;      /* ��ǰI֡���������Ƶ֡����ı�� */
    unsigned int resolve[2];     /* �����ֶ�                        */
}__attribute__((packed))IFrameInfo, *pIFrameInfo;

static global_head_t g_global_head;                 // ȫ������ͷ(64Bytes)

static Avi_file_info g_avi_file_set[MAX_AVI_NUM];   // ��ȫ��������ȡ����AVI�ļ���Ϣ����(625KBytes)

static int g_curr_avi_num = 0;                      // ��ǰȫ�������������ЧAVI����

/*@breif BinareySearchEx ���ַ����ҵ���������ӽ�����
 *@param [IN] data �����ҵĵ��������
 *@param [IN] data_count �������Ԫ�ظ���
 *@param [IN] key �����ҵ���
 *@return �ɹ����������±�
 *@       ʧ�ܷ���-1
 *@note   ֧���ظ�����, ����������������±��һ
 */
inline int BinareySearchEx(int *data, int data_count, int key)
{
    if(!data  || data_count < 1)
    {
        printf("%s %d error !\n", __func__, __LINE__);
        return -1;      
    }

    if(data_count == 1)
    {         
        return data_count-1;
    }  

    if(key <= data[0])   
    {  
        printf("key = %d, data[0] = %d\n", key, data[0]);            
        return 0;  
    }     
    else if(key >= data[data_count-1])    
    {       
        printf("key = %d, data[%d] = %d\n", key, data_count -1, data[data_count-1]); 
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

    return start-data-1; /* ���ز�����������±� */
}

int cmp_avi_start_time(const void *a, const void *b)
{
    Avi_file_info *aa = (Avi_file_info *)a;
    Avi_file_info *bb = (Avi_file_info *)b;
    return ((aa->start_time)>(bb->start_time)?1:-1);
}

int com_avi_i_frame_time(const void *a, const void *b)
{
    IFrameInfo *aa = (IFrameInfo *)a;
    IFrameInfo *bb = (IFrameInfo *)b;
    return ((aa->CurIFrameTime)>(bb->CurIFrameTime)?1:-1);
}

/* ��ȫ���ļ������ȡ��Ϣͷ��AVI��Ϣ����ȫ�ֽṹ������ */
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
    
    /* ��λ���ļ�ͷ����ȡ64�ֽڵ�ȫ������ͷ */
    lseek(fd, 0, SEEK_SET);
    int read_size = read(fd, &g_global_head, sizeof(g_global_head));
    if (sizeof(global_head_t) != read_size)
    {
        SD_ERR("read global head info fail!\n");
        close(fd);
        return -1;
    }

    int *p = (int *)&g_global_head;

    /* У��ṹ�����ȷ�� */
    if (g_global_head.head_SEI != 0x7765767a 
     || g_global_head.head_verify != 0x1002feea 
     || g_global_head.struct_xor_crc != XOR_INT_64BYTES(p))
    {
        SD_ERR("global head info crc fail!\n");
        close(fd);
        return -1;
    }

    int i = 0;
    
    /* ��ȫ���ļ������ȡÿ��avi����Ϣ���ڽṹ������ */
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

/* ��avi�ļ������ȡI֡��Ϣ����ھ������ */
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

    /* ��ȡAVIͷ���������ȡAVI���ݴ�С(������I֡��Ϣ�Ĵ�AVI���ݴ�С) */
    lseek(fd, 4, SEEK_SET);
    if (4 != read(fd, data, 4))
    {
        SD_ERR("read avi_size fail!\n");
        close(fd);
        return -1;
    }

    int avi_size = ((data[0]&0xff)|((data[1]&0xff)<<8)|((data[2]&0xff)<<16)|((data[3]&0xff)<<24)) + 8;
    
    /* �����ļ�ָ�뵽����AVI����β�� */
    lseek(fd, avi_size, SEEK_SET);

    /* ֱ�Ӷ�ȡ16KB��INFO��Ϣ��С */
    read(fd, i_frame_list, MAX_I_FRAME_NUM*sizeof(IFrameInfo));

    close(fd);
    fd = -1;

    qsort(i_frame_list, MAX_I_FRAME_NUM, sizeof(IFrameInfo), com_avi_i_frame_time);

    return 0;    
}

/* �Ӹ�����ʱ��������ȡ���ϸ�ʱ��ε�avi�ļ��±� */
int GetTimeSegmentArray(int start_time, int stop_time, int *seg_start_index, int *seg_stop_index)
{
    if (!seg_start_index || !seg_stop_index)
    {
       SD_ERR("NULL POINTER EXCEPTION!\n");
       return -1;
    }

    /* ����ʱ���޶����ܳ���24H */
    if (stop_time - start_time > 24*60*60)
    {
        SD_ERR("Find time segment too long! not more than 24h!\n");
        return -1;
    }

    start_time = start_time<g_avi_file_set[0].start_time?g_avi_file_set[0].start_time:start_time;
    stop_time  = stop_time>g_avi_file_set[g_curr_avi_num-1].stop_time?g_avi_file_set[g_curr_avi_num-1].stop_time:stop_time;

    int i = 0;
    int curr_avi_times[MAX_FOUND_AVI_NUM] = {0};

    for (i=0; i<g_curr_avi_num; i++)
    {
        curr_avi_times[i] = g_avi_file_set[i].start_time;
    }

    *seg_start_index = BinareySearchEx(curr_avi_times, g_curr_avi_num, start_time);
    *seg_stop_index  = BinareySearchEx(curr_avi_times, g_curr_avi_num, stop_time);

    return 0;
}

/* �Ӹ������ļ�����ṹ�������ȡ��Ӧ��AVI�ļ�������ȡ��Ӧ��I֡֡�� */
int SeekTimePointFromAviFileList(int seek_time, int seg_start_index, int seg_stop_index, int *get_iframe_num)
{
    if (!get_iframe_num)
    {
       SD_ERR("NULL POINTER EXCEPTION!\n");
       return -1;
    }

    if (seg_start_index > seg_stop_index)
    {
        SD_ERR("Segment index [%d > %d] fail!\n", seg_start_index, seg_stop_index);
        return -1;
    }

    int i = 0, j = 0, ret = -1;
    int avi_start_time_list[MAX_FOUND_AVI_NUM] = {0};
    IFrameInfo iframe_list[MAX_I_FRAME_NUM];

    for (i=seg_start_index, j=0; i<=seg_stop_index; i++, j++)
    {
        avi_start_time_list[j] = g_avi_file_set[i].start_time;
    }

    /* ���ַ����һ�ȡ����ӽ����ļ��� */
    i = BinareySearchEx(avi_start_time_list, j, seek_time);

    SD_DBG("found avi name is %s\n", g_avi_file_set[i+seg_start_index].file_name);

    /* �ӵ�ǰAVI�ļ������ȡ��ӽ���I֡֡�� */
    char avi_file[64] = {0};
    sprintf(avi_file, "%s/video/%s", SD_MOUNT_POINT, g_avi_file_set[i+seg_start_index].file_name);

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
    int start_time = 0;
    int stop_time  = 0;
    int get_iframe_num = 0;
    unsigned int spent_time_ms = 0;

    DECLARE_SHOT_CLOCK();

    START_SHOT_CLOCK();

    ret = GetGlobalHeadInfoFromGlobalFile();
    if (ret)
    {
        SD_ERR("Fail to called GetGlobalHeadInfoFromGlobalFile\n");
        return -1;
    }

    ret = GetTimeSegmentArray(atoi(argv[1]), atoi(argv[2]), &start_time, &stop_time);
    if (ret)
    {
        SD_ERR("Fail to called GetTimeSegmentArray\n");
        return -1;
    }
    
    ret = SeekTimePointFromAviFileList(atoi(argv[3]), start_time, stop_time, &get_iframe_num);
    if (ret)
    {
        SD_ERR("Fail to called SeekTimePointFromAviFileList\n");
        return -1;
    }
    STOP_SPEND_SHOT_CLOCK(spent_time_ms);

    SD_INF("Found iframe num is %d\n", get_iframe_num);

    SD_INF("Found info spent %d ms\n", spent_time_ms/1000);

    return 0;
}