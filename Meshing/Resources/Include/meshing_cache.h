/* SPDX-License-Identifier: Apache-2.0 */
#ifndef COGNIPILOT_MESHING_CACHE_H
#define COGNIPILOT_MESHING_CACHE_H
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ModelicaUtilities.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

/* SHA-256, FIPS 180-4. The input is the recipe followed by NUL and the file bytes. */
typedef struct { uint32_t h[8]; uint64_t bytes; size_t used; unsigned char block[64]; } MeshDigest;
static uint32_t mesh_rotr(uint32_t x, unsigned n) { return (x >> n) | (x << (32-n)); }
static void mesh_transform(MeshDigest *s) {
    static const uint32_t k[64]={
      0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
      0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
      0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
      0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
      0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
      0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
      0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
      0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
    uint32_t w[64], a=s->h[0],b=s->h[1],c=s->h[2],d=s->h[3],e=s->h[4],f=s->h[5],g=s->h[6],h=s->h[7];
    for (int i=0;i<16;i++) {
        const unsigned char *p=s->block+4*i;
        w[i]=((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
    }
    for (int i=16;i<64;i++) {
        uint32_t x=w[i-15],y=w[i-2];
        w[i]=w[i-16]+(mesh_rotr(x,7)^mesh_rotr(x,18)^(x>>3))+w[i-7]+(mesh_rotr(y,17)^mesh_rotr(y,19)^(y>>10));
    }
    for (int i=0;i<64;i++) {
        uint32_t t=h+(mesh_rotr(e,6)^mesh_rotr(e,11)^mesh_rotr(e,25))+((e&f)^(~e&g))+k[i]+w[i];
        uint32_t u=(mesh_rotr(a,2)^mesh_rotr(a,13)^mesh_rotr(a,22))+((a&b)^(a&c)^(b&c));
        h=g;g=f;f=e;e=d+t;d=c;c=b;b=a;a=t+u;
    }
    s->h[0]+=a;s->h[1]+=b;s->h[2]+=c;s->h[3]+=d;s->h[4]+=e;s->h[5]+=f;s->h[6]+=g;s->h[7]+=h;
}
static void mesh_update(MeshDigest *s,const unsigned char *data,size_t length) {
    s->bytes+=length;
    while (length) {
        size_t take=64-s->used;if (take>length) take=length;
        memcpy(s->block+s->used,data,take);s->used+=take;data+=take;length-=take;
        if (s->used==64) { mesh_transform(s);s->used=0; }
    }
}
static int mesh_digest_file(const char *path,const char *recipe,char out[65]) {
    MeshDigest s={{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19},0,0,{0}};
    unsigned char buffer[16384],pad[128]={0x80};
    size_t count;
    FILE *file=fopen(path,"rb");
    if (!file) return 0;
    mesh_update(&s,(const unsigned char *)recipe,strlen(recipe)+1);
    while ((count=fread(buffer,1,sizeof(buffer),file))>0) mesh_update(&s,buffer,count);
    int ok=!ferror(file);fclose(file);if (!ok) return 0;
    uint64_t bits=s.bytes*8;
    size_t padding=s.used<56?56-s.used:120-s.used;
    for (int i=0;i<8;i++) pad[padding+7-i]=(unsigned char)(bits>>(8*i));
    mesh_update(&s,pad,padding+8);
    static const char hex[]="0123456789abcdef";
    for (int i=0;i<64;i++) out[i]=hex[(s.h[i/8]>>(28-4*(i%8)))&15];
    out[64]=0;return 1;
}
static const char *mesh_key(const char *path,const char *recipe) {
    char *key=ModelicaAllocateString(64);
    if (!mesh_digest_file(path,recipe,key)) ModelicaFormatError("Cannot hash mesh input %s",path);
    return key;
}
static const char *mesh_temp(const char *path) {
#ifdef _WIN32
    char folder[MAX_PATH],name[MAX_PATH];
    DWORD length=GetFullPathNameA(path,MAX_PATH,folder,NULL);
    if (length==0 || length>=MAX_PATH || !strrchr(folder,'\\')) ModelicaError("Invalid mesh cache directory");
    strrchr(folder,'\\')[1]=0;
    if (!GetTempFileNameA(folder,"msh",0,name)) ModelicaError("Cannot create temporary mesh file");
    char *result=ModelicaAllocateString(strlen(name));strcpy(result,name);return result;
#else
    char *result=ModelicaAllocateString(strlen(path)+7);
    sprintf(result,"%s.XXXXXX",path);
    int fd=mkstemp(result);
    if (fd<0) ModelicaFormatError("Cannot create temporary mesh beside %s",path);
    close(fd);return result;
#endif
}
static void mesh_publish(const char *tmp,const char *path) {
#ifdef _WIN32
    int ok=MoveFileExA(tmp,path,MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH)!=0;
#else
    int ok=rename(tmp,path)==0;
#endif
    if (!ok) { remove(tmp);ModelicaFormatError("Cannot publish mesh file %s",path); }
}
static int mesh_verified(const char *path,const char *signature,const char *key) {
    FILE *f=fopen(signature,"rb");if (!f) return 0;
    char expected[65],actual[65];
    int ok=fread(expected,1,64,f)==64 && fgetc(f)==EOF;
    fclose(f);
    return ok && mesh_digest_file(path,key,actual) && memcmp(expected,actual,64)==0;
}
static void mesh_sign(const char *path,const char *signature,const char *key) {
    const char *hash=mesh_key(path,key), *tmp=mesh_temp(signature);
    FILE *f=fopen(tmp,"wb");
    if (!f) ModelicaError("Cannot write mesh signature");
    int ok=fwrite(hash,1,64,f)==64;
    if (fclose(f)!=0 || !ok) { remove(tmp);ModelicaError("Incomplete mesh signature"); }
    mesh_publish(tmp,signature);
}

/* Uncompressed Level-4 MAT matrices, in native IEEE byte order and column order. */
static int mesh_mat_header(FILE *file,const char *name,int rows,int cols) {
    uint16_t one=1;
    int32_t h[5]={*(const unsigned char *)&one ? 0 : 1000, rows, cols, 0, (int32_t)strlen(name)+1};
    return fwrite(h,sizeof(*h),5,file)==5 && fwrite(name,1,h[4],file)==(size_t)h[4];
}
static int mesh_int_column(FILE *file,const int *values,int n) {
    double buffer[1024];
    for (int i=0;i<n;) {
        int count=n-i<1024?n-i:1024;
        for (int j=0;j<count;j++) buffer[j]=values[i+j];
        if (fwrite(buffer,sizeof(double),count,file)!=(size_t)count) return 0;
        i+=count;
    }
    return 1;
}
static void mesh_write_mat(const char *tmp,int np,int nf,int nk,int nc,const double *points,
                           const int *first,const int *owner,const int *other,const int *patch,const int *corners) {
    FILE *file=fopen(tmp,"wb");
    if (!file) ModelicaFormatError("Cannot open mesh file %s",tmp);
    double header[5]={1,np,nf,nk,nc},buffer[1024];
    int ok=mesh_mat_header(file,"header",1,5) && fwrite(header,sizeof(double),5,file)==5
          && mesh_mat_header(file,"points",np,3);
    for (int col=0;col<3 && ok;col++) {
        for (int i=0;i<np && ok;) {
            int count=np-i<1024?np-i:1024;
            for (int j=0;j<count;j++) buffer[j]=points[3*(size_t)(i+j)+col];
            ok=fwrite(buffer,sizeof(double),count,file)==(size_t)count;
            i+=count;
        }
    }
    ok=ok && mesh_mat_header(file,"faces",nf,4) && mesh_int_column(file,first,nf)
          && mesh_int_column(file,owner,nf) && mesh_int_column(file,other,nf) && mesh_int_column(file,patch,nf)
          && mesh_mat_header(file,"corners",nk,1) && mesh_int_column(file,corners,nk);
    if (fclose(file)!=0 || !ok) { remove(tmp);ModelicaFormatError("Failed writing complete mesh to %s",tmp); }
}
#endif
