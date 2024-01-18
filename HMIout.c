#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define SOURCE_FILE "logs/ECU.log"

void HMIoutput();
void checkLine(char[], FILE*);
bool checkFileChange(int);
unsigned int sleep(int);

int main() {
	printf("///HMI OUTPUT TERMINAL///\n");
	sleep(1);
	HMIoutput();
}

void HMIoutput() { 
	FILE *srcFile = fopen(SOURCE_FILE, "r");
	if (srcFile != NULL) {
		char line[256];
		while (fgets(line, sizeof(line), srcFile) != NULL) {
			printf("ECU: %s", line);
			checkLine(line, srcFile);
			sleep(1);
		}
		fclose(srcFile);
	} else {
		printf("Impossibile aprire il file di log per la lettura dei comandi!\n");
	}
	
}

void checkLine(char line[], FILE* srcFile) {
	if (strcmp(line, "NAVIGAZIONE INTERROTTA\n") == 0 || strcmp(line, "AVVIO PARCHEGGIO EFFETTIVO\n") == 0) {
		int lastPos = ftell(srcFile);
		while(checkFileChange(lastPos)) {
			sleep(1);
		}
	} 
}

bool checkFileChange(int lastPos) {
	FILE *srcFile = fopen(SOURCE_FILE, "r");
	fseek(srcFile, 0, SEEK_END);
	return (ftell(srcFile) == lastPos);
}

