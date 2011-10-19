#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h> 
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define FILE_MODE   (S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH)
int main(int argc, char *argv[])
{
    	int         fdin, fdout;
    	unsigned char        *src, *dst;
    	struct stat statbuf;
    	int 	i,j,size,offset;
    	
	    long long pos;
    	
    	if (argc > 5)
	  	{
        	printf("usage: %s <fromfile> <tofile> <offset>\n", argv[0]);
			exit(1);	  
		}

		printf("args:1,%s;2,%s;3,%s\n",argv[1],argv[2],argv[3]);
    	if ((fdin = open(argv[1], O_RDONLY)) < 0)
		{
        	printf("can't open %s for reading", argv[1]);
			exit(1);
		}

    	if ((fdout = open(argv[2], O_RDWR | O_CREAT,
      	FILE_MODE)) < 0)
		{
        	printf("can't creat %s for writing", argv[2]);
			exit(1);
		}

    	if (fstat(fdin, &statbuf) < 0)   /* need size of input file */
		{
        	printf("fstat error");
			exit(1);		
		}
		offset=strtoul(argv[3],NULL,0);
		if(offset %2048)
		{
			printf("offset not a multiple of 2048 bytes\n");
			exit(1);
		}
    	size=((statbuf.st_size+offset+2047)/2048)*2112;
        /* set size of output file */
    	if (lseek(fdout, size - 1, SEEK_SET) == -1)
		{
		  	printf("lseek error");
			exit(1);
		}
    	if (write(fdout, "", 1) != 1)
		{
        	printf("write error");
			exit(1);
		}
    	if ((src = mmap(0, statbuf.st_size, PROT_READ, MAP_SHARED,
      	fdin, 0)) == MAP_FAILED)
		{
        	printf("mmap error for input");
			exit(1);
		}

    	if ((dst = mmap(0, size, PROT_READ | PROT_WRITE,
      	MAP_SHARED, fdout, 0)) == MAP_FAILED)
		{
        	printf("mmap error for output");
			exit(1);
		}
    	//memset(dst+offset,0xFF,size-offset);
	    
	    printf("offset: 0x%x = %lld \n",offset, (unsigned long long)offset );
	    
	    pos = ( long long)offset * ( long long)2112 ;
	    printf("1  pos: 0x%x = %lld \n",pos, (long long)pos );
	    
	    pos = ( long long) pos / ( long long)2048;
        printf("2  pos: 0x%x = %lld \n",pos, ( long long)pos );
	    
		if (argv[4] != NULL && strcmp(argv[4],"yaffs") == 0)
		{
			printf("yaffs write\n");
			for(i=0; i<statbuf.st_size; i++,pos++)
			{
				*(dst+pos)=*(src+i);
			}

		}
		else
		{
			for(i=0; i<statbuf.st_size; i++,pos++)
			{
				if((i%2048)==0&&i) 
					for(j=0;j<64;j++)
					{
						*(dst+pos)=0xFF;
						pos++;		
					}
				*(dst+pos)=*(src+i);
			}
		}

    	msync(dst,size,MS_SYNC);
    	printf("finish\n");
    	exit(0);
}
