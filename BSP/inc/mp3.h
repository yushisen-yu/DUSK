#ifndef _MP3_H_
#define _MP3_H_

#ifdef __cplusplus
extern "C" {
#endif

#define UDISK   0
#define FLASH0   4

void setMp3Dev(unsigned char dev);
void setMp3Vol(unsigned char vol);
void mp3Play(void);
void mp3Stop(void);
void mp3_next();
void mp3_prev();
void mp3_play_selected(unsigned short index);


#ifdef __cplusplus
}
#endif
#endif
