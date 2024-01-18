#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <time.h>
#include <ctype.h>
#include <fcntl.h>
#include <dirent.h>
#include <signal.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#define HMIOUTPUT_LAUNCH_COMMAND "gnome-terminal -- bash -c './HMIout; exec bash'"
#define MAIN_CAMERA_SRC "data/frontCameraCustom2.data"

#define MAIN_LOG_FILE "logs/ECU.log"
#define CAMERA_LOG "logs/camera.log"
#define RADAR_LOG "logs/radar.log"
#define THROTTLE_LOG "logs/throttle.log"
#define BRAKE_LOG "logs/brake.log"
#define STEER_LOG "logs/steer.log"
#define ASSIST_LOG "logs/assist.log"
#define CAMERAS_LOG "logs/cameras.log"

// GLOBAL VARIABLES //
int currentSpeed = 0;
bool interruptedNav = true;
bool HMITerminalOpen = false;
char *sensorsSource;
int hmiServerFD;
int navDataCursor = 0; // PER RINIZIARE DAL PUNTO GIUSTO DOPO PERICOLO

// IMPLEMENTED FUNCTIONS HEADERS //
bool validInput(char[]);
void navigation(int[]);
int evalData(char[], pid_t[], int, int, int, int);

int checkThrottleFailure();
int speedUp(char[]);
int slowDown(char[]);
void steerCar(char[]);
int steerPosition(char[], int);
void sigusr1Handler(int);
void sigusr2Handler(int);
void sigTermParkHandler(int);
bool preParkBraking(int);
void parkCar();
int activateSurroundCameras();
bool bytesCheckFailure(char[]);

//logging
void logServerEvent(char[], bool);
void logSteerEvent(char[]);
void logThrottleEvent();
void logBrakeEvent(char[]);
void logParkingEvent(FILE*, char[]);
int clearLogFiles();


int main(int argc, char *argv[]) {
	char* fileName = argv[0];
	if (argc < 2) {
        	printf("Utilizzo: %s <modalità> (<NORMALE/ARTIFICIALE>)", fileName);
        	exit(1);
    	}
    	char* mode = argv[1];	

	if (strcmp(mode, "NORMALE") == 0 || strcmp(mode, "ARTIFICIALE") == 0) {
		if (strcmp(mode, "NORMALE") == 0) {
			sensorsSource = "/dev/urandom";
		} else {
			sensorsSource  = "data/urandomARTIFICIALE.binary";
		}
		int logsCleared = clearLogFiles();
		printf("Avvio ECU in modalità -> %s\nI sensori leggeranno da '%s'\n", mode, sensorsSource);
		if (logsCleared == -8) {
			printf("Nessun file di log presente, Avvio del server...\n");
		} else {
			printf("Trovati e rimossi %d file di log esistenti...\nAvvio del server...\n", logsCleared+8);
		}
    		
		int sockfd[2]; 
        	char buffer[20];
		if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockfd) == -1) {
			perror("Errore nella creazione della coppia di socket tra il server e l'HMI input");
			exit(1);
		}
		
		pid_t hmi_pid = fork();
		
		if (hmi_pid < 0) {
			perror("Errore nella creazione del processo HMI");
			exit(1);
		} else if (hmi_pid == 0) {
			// (main client: HMI)
			sleep(3);
			close(sockfd[0]); 
			while (1) {
				char inputHMI[20]; 
				printf("HMI in attesa di un input (INIZIO/PARCHEGGIO/ARRESTO): \n");
				scanf("%s", inputHMI);
				if (validInput(inputHMI)) {
					printf("Invio di %s alla ECU...\n", inputHMI);
					write(sockfd[1], inputHMI, strlen(inputHMI));
				} else {
					printf("Input sconosciuto(%s)!\n", inputHMI); 
				}
				if (kill(getppid(), 0) == -1) {
					break;
				}
				sleep(2);
			}
			close(sockfd[1]); 
			exit(0);
		} else {
			// (server: ECU)
			close(sockfd[1]);
			printf("Server avviato!\n");
			int bytes_received = read(sockfd[0], buffer, sizeof(buffer));
			while (bytes_received > 0) {
				buffer[bytes_received] = '\0';
				if (strcmp(buffer, "INIZIO") == 0) {
					if (interruptedNav) {
					    	int sockCamfd[2];
					    	if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockCamfd) == -1) {
							perror("Errore nella creazione della socket tra il server e front windshield camera!");
							exit(1);
						}
					    	hmiServerFD = sockCamfd[1];
					    	interruptedNav = false;
			
					    	pid_t nav_pid = fork();
						
						if (nav_pid < 0) {
							perror("Errore nella creazione del processo che gestisce la navigazione!");
							exit(1);
						} else if (nav_pid != 0) { // NAVIGATION PROCESS AS FATHER //
							printf("Avvio navigazione...\nAvvio dei componenti in corso...\n");
							if (!HMITerminalOpen && system(HMIOUTPUT_LAUNCH_COMMAND) != 0) {
								printf("Errore nella creazione della terminale di output di HMI!\nAssicurarsi di avere 'gnome-terminal installato!\nA");
							} else {
								HMITerminalOpen = true;
							}
						    	navigation(sockCamfd);  // NAVIGATION PROCESS //
						    	printf("Navigazione interrotta!\nImmettere INIZIO per riavviarla:\n");
						    	interruptedNav = true;
						  	kill(nav_pid, SIGTERM);
						} 
					} else {
						printf("La navigazione è già in corso!\n");
					}
				} else if (strcmp(buffer, "PARCHEGGIO") == 0) {
					if (!interruptedNav) {
						write(hmiServerFD, buffer, strlen(buffer));
					} else {
						printf("Parcheggio non eseguibile, l'auto è già parcheggiata!\n");
					}
				} else if (strcmp(buffer, "ARRESTO") == 0) {
					if (!interruptedNav) {
						write(hmiServerFD, buffer, strlen(buffer)); 
					} else {
						printf("Arresto non eseguibile, l'auto è già ferma!\n");
					}
				}
				if (kill(getppid(), 0) == -1) {
					break;
				}
				bytes_received = read(sockfd[0], buffer, sizeof(buffer));
				
			}
			close(sockfd[0]); 
    		}
	} else {
		printf("Modalità di avvio sconosciuta, usare solo (<NORMALE/ARTIFICIALE>)"); 
		exit(1); 
	}
}

bool validInput(char inputHMI[]) {
	return (strcmp(inputHMI, "INIZIO") == 0 || strcmp(inputHMI, "PARCHEGGIO") == 0 || strcmp(inputHMI, "ARRESTO") == 0);
}

void navigation(int sockCamfd[2]) { 
	pid_t frontCam_pid = fork();
	
	if (frontCam_pid < 0) {
		perror("Errore nella creazione del processo front windshield camera!");
		exit(1);
	} else if (frontCam_pid == 0) { // WINDSHIELD CAM PROCESS //
		close(sockCamfd[0]);
		FILE *dataFile = fopen(MAIN_CAMERA_SRC, "r");
		FILE *logFile = fopen(CAMERA_LOG, "a");
		if (dataFile != NULL && logFile != NULL && fseek(dataFile, navDataCursor, SEEK_CUR) == 0) {
			char line[30];
			while (fgets(line, sizeof(line), dataFile) != NULL) {
				write(sockCamfd[1], line, strlen(line));
				fprintf(logFile, "%s", line);
				if (strcmp(line, "DESTRA") == 0 || strcmp(line, "SINISTRA") == 0) {
					sleep(5);
				} else {
					sleep(1); // should be 1 sec //
				}
			}
			fclose(dataFile);
			fclose(logFile);
		} 
		close(sockCamfd[1]);
		exit(0);
	} else {
		int sockThrottlefd[2];
		if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockThrottlefd) == -1) {
			perror("Errore nella creazione della socket tra il server e throttle control");
			exit(1);
		} 
		
		pid_t throttle_pid = fork();

		if (throttle_pid < 0) {
			perror("Errore nella creazione del processo throttle control!");
			exit(1);
		} else if (throttle_pid == 0) {  // THROTTLE PROCESS //
			sleep(1);
			char bufferThrottle[256];
			int throttle_bytes = read(sockThrottlefd[1], bufferThrottle, sizeof(bufferThrottle));
			while (throttle_bytes > 0) {
				bufferThrottle[throttle_bytes] = '\0';
				sprintf(bufferThrottle, "%d", speedUp(bufferThrottle));
				write(sockThrottlefd[1], bufferThrottle, strlen(bufferThrottle));
				throttle_bytes = read(sockThrottlefd[1], bufferThrottle, sizeof(bufferThrottle));
			}
			close(sockThrottlefd[1]);
			exit(0);
		} else {
			int sockBrakefd[2];
			if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockBrakefd) == -1) {
				perror("Errore nella creazione della socket tra il server e brake by wire");
				exit(1);
			} 
			
			pid_t brake_pid = fork();
			
			if (brake_pid < 0) {
				perror("Errore nella creazione del processo brake by wire!");
				exit(1);
			} else if (brake_pid  == 0) {  // BRAKE PROCESS //
				sleep(1);
				char bufferBrake[256];
				signal(SIGUSR1, sigusr1Handler); //ARRESTO-PERICOLO//
				int brake_bytes = read(sockBrakefd[1], bufferBrake, sizeof(bufferBrake));
				while (brake_bytes > 0) {
					bufferBrake[brake_bytes] = '\0';
					sprintf(bufferBrake, "%d", slowDown(bufferBrake));
					write(sockBrakefd[1], bufferBrake, strlen(bufferBrake));
					brake_bytes = read(sockBrakefd[1], bufferBrake, sizeof(bufferBrake));
				}
				close(sockBrakefd[1]);
				exit(0);
			} else {
				int sockSteerfd[2];
				if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockSteerfd) == -1) {
					perror("Errore nella creazione della socket tra il server e steer by wire");
					exit(1);
				}
				
				pid_t steer_pid = fork();
				
				if (steer_pid  < 0) {
					perror("Errore nella creazione del processo brake by wire!");
					exit(1);
				} else if (steer_pid  == 0) {  // STEER PROCESS //
					sleep(1);
					char bufferSteer[256];
					int steer_bytes = read(sockSteerfd[1], bufferSteer, sizeof(bufferSteer));
					while(steer_bytes > 0) {
						bufferSteer[steer_bytes] = '\0';
						steerCar(bufferSteer);
						steer_bytes = read(sockSteerfd[1], bufferSteer, sizeof(bufferSteer));
					}
					close(sockSteerfd[1]);
					exit(0);
				} else {
					int sockRadarfd[2];
					if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockRadarfd) == -1) {
						perror("Errore nella creazione della socket tra il server e forward facing radar");
						exit(1);
					}

					pid_t radar_pid = fork();
							
					if (radar_pid < 0) {
						perror("Errore nella creazione del processo forward facing radar!");
						exit(1);
					} else if (radar_pid == 0) {  // RADAR PROCESS //
						sleep(1);
						int sourceFD = open(sensorsSource, O_RDONLY);
						FILE *logFile= fopen(RADAR_LOG, "a");
						if (sourceFD != -1 && logFile != NULL) {
							char bufferRadar[8];
							ssize_t bytesRead = read(sourceFD, bufferRadar, sizeof(bufferRadar));
							while(bytesRead != -1) {
								sleep(1);
								bytesRead = read(sourceFD, bufferRadar, sizeof(bufferRadar));
								if (bytesRead != sizeof(bufferRadar)) {
									continue;
								} 
								write(sockRadarfd[1], bufferRadar, strlen(bufferRadar));
								for (int i = 0; i < sizeof(bufferRadar); i++) {
        								fprintf(logFile, "%02x ", bufferRadar[i]);
					    			}
					    			fprintf(logFile, "\n");
					    			fflush(logFile);
							}
							fclose(logFile);
							close(sourceFD);
							close(sockRadarfd[1]);
							exit(0);
						} else {
							printf("Errore nell'apertura di %s o di %s\n! Forward facing radar non attivo!", sensorsSource, RADAR_LOG);
						}
					} else {
					  	pid_t components_pids[] = {frontCam_pid, throttle_pid, brake_pid, steer_pid, radar_pid};  // NAVIGATION MAIN PROCESS //
					  	int lastSpeed;
						bool quitNav = false;
					  	char buffer[256];
					  	signal(SIGUSR2, sigusr2Handler); //FALLIMENTO THROTTLE//
						int cam_bytes = read(sockCamfd[0], buffer, sizeof(buffer));
						
						printf("Navigazione avviata!\n");
						logServerEvent("NAVIGAZIONE AVVIATA ---> ", true);
						while (cam_bytes > 0) {
							buffer[cam_bytes] = '\0';
							lastSpeed = currentSpeed;
							currentSpeed += evalData(buffer, components_pids, sizeof(components_pids), sockThrottlefd[0], sockBrakefd[0], sockSteerfd[0]);
							//printf("Buffer: %s Actual: %d\n", buffer, currentSpeed);
							if (currentSpeed != -1 && currentSpeed != -2 && currentSpeed != -3 && currentSpeed != -4) { 
								if (lastSpeed == currentSpeed) {
									logServerEvent("PROCEDO ---> ", true);
								} else if (lastSpeed < currentSpeed) {
									logServerEvent("INCREMENTO 5 ---> ", true);
								} else if (lastSpeed > currentSpeed) {
									logServerEvent("FRENO 5 ---> ", true);
								}
							} else {
								switch(currentSpeed) {
									case -1: // ARRESTO COME IMPUT CHE NON INTERROMPE			
										currentSpeed = 0;
										printf("L'auto è stata fermata!\nRiprendo la navigazione...\n");
										logServerEvent("ARRESTO AUTO (INPUT) ---> ", true);
										break;
									case -2: // ARRESTO COME PERICOLO 					
										currentSpeed = 0;
										for (int i = sizeof(components_pids); i == 0; i--) { 
											kill(components_pids[i], SIGTERM);
										}
										logServerEvent("ARRESTO AUTO (PERICOLO) ---> ", true);
										logServerEvent("NAVIGAZIONE INTERROTTA", false);
										quitNav = true;
										break;
									case -3: 
										currentSpeed = lastSpeed;
										for (int i = 0; i < 4; i++) {
											logServerEvent("STERZANDO ---> ", true);
											sleep(1);
										}
										break;
									case -4: 
										sleep(1);
										currentSpeed = 0;
										logServerEvent("AUTO PARCHEGGIATA", false);
										printf("Procedura di parcheggio completata!\nConcludo la navigazione...\n");
										for (int i = sizeof(components_pids); i == 0; i--) { 
											kill(components_pids[i], SIGTERM);
										}
										logServerEvent("NAVIGAZIONE CONCLUSA", false);
										kill(getpid(), SIGTERM);
										break;
								}
							} 
							if (quitNav) {
								cam_bytes = -1;
							} else {
								cam_bytes = read(sockCamfd[0], buffer, sizeof(buffer));
							}
						}
						close(sockCamfd[0]);
					}		
				}
			}
		}
	}
}

int evalData(char bufferCam[], pid_t pids[], int n_pids,int sockThrottleFd, int sockBrakeFd, int sockSteerFd) {
	char *comand;
	char deltaSpeed[5];
	int requestedSpeed;
	
	if (strcmp(bufferCam, "DESTRA\n") == 0) {
		navDataCursor += 7;
		return steerPosition(bufferCam, sockSteerFd);
	} else if (strcmp(bufferCam, "SINISTRA\n") == 0) {
		navDataCursor += 9;
		return steerPosition(bufferCam, sockSteerFd);
	} else {
		steerPosition("NEUTRO\n", sockSteerFd);
	}
	
	if (strcmp(bufferCam, "PARCHEGGIO\n") == 0 || strcmp(bufferCam, "PARCHEGGIO") == 0 ) { //RIMETTERE PARCHEGGIO CONDITION
		navDataCursor += 10;
		if (preParkBraking(sockBrakeFd)) {
			logServerEvent("AVVIO PARCHEGGIO EFFETTIVO", false);
			printf("Auto fermata! Sto parcheggiando...\n");
			parkCar(); // actual parking 
			return (currentSpeed*-1)-4;
		} 
	} else if (strcmp(bufferCam, "PERICOLO\n") == 0) {
		navDataCursor += 9;
		if (kill(pids[2], SIGUSR1) == 0) { // arrest car caused by pericolo
			sleep(1); // wait for signal handler
			return (currentSpeed*-1)-2; 
		} else {
			printf("Errore nell'invio del segnale di pericolo a brake by wire\n");
		}
	} else if (strcmp(bufferCam, "ARRESTO") == 0) {
		if (kill(pids[2], SIGUSR1) == 0) { // arrest car caused by user input
			sleep(1); // wait for signal handler
			return (currentSpeed*-1)-1;
		} else {
			printf("Errore nell'invio del segnale di arresto a brake by wire\n");
		}
	} 
	
	if (isdigit(bufferCam[0]) > 0) {
		navDataCursor += 3;
		requestedSpeed = atoi(bufferCam);
		if (requestedSpeed > currentSpeed) {
			comand = "Incremento 5";
			write(sockThrottleFd, comand, strlen(comand));
			read(sockThrottleFd, deltaSpeed, sizeof(deltaSpeed));
		} else if (requestedSpeed < currentSpeed) {
			comand = "Freno 5";
			write(sockBrakeFd, comand, strlen(comand));
			read(sockBrakeFd, deltaSpeed, sizeof(deltaSpeed));
		} else {
			return 0;
		}
		int speedChange = atoi(deltaSpeed);
		if (speedChange == -1) { 
			exit(0); // arrest car caused by throttle failure, this line is never reached
		} else {
			return speedChange;
		}
	}
	return 0;
}

int checkThrottleFailure() {
	const double probability = 1e-5; 
	double outcome = (double) rand() / RAND_MAX;
	return (outcome < probability) ? 1 : 0;
}

int speedUp(char comand[]) { 
	if (strcmp(comand, "Incremento 5") == 0) {
		if (checkThrottleFailure()) {
			printf("Throttle control ha appena fallito!\n");
			kill(getppid(), SIGUSR2);
			return -1;
	        } else {
			logThrottleEvent();
                }
                return 5;
        } else {
        	printf("Comando errato, scrivere nel formato 'Incremento 5'\n");
        }
}

int slowDown(char comand[]) {	
	if (strcmp(comand, "Freno 5") == 0) {
		logBrakeEvent("FRENO 5");
		return -5;
	} else {
        	printf("Comando errato, scrivere nel formato 'Freno 5'\n");
        }
} 

void steerCar(char direction[]) {
	int count = 0;
	if (strcmp(direction, "DESTRA\n") == 0){
		while (count < 4) {
			logSteerEvent(direction);
			count++;
			sleep(1);
		}
	} else if (strcmp(direction, "SINISTRA\n") == 0){
		while (count < 4) {
			logSteerEvent(direction);
			count++;
			sleep(1);
		}
    	} else if (strcmp(direction, "NEUTRO\n") == 0) {
    		logSteerEvent("NO ACTION\n");
    	} 
}

int steerPosition(char comand[], int sockSteerFd) {
	write(sockSteerFd, comand, strlen(comand));
	return (currentSpeed*-1)-3;
}

bool preParkBraking(int sockBrakeFd) {
	char buffer[5];
	logServerEvent("AVVIO PROCEDURA PARCHEGGIO ---> ", true);
	printf("Fermo l'auto per avviare la procedura di parcheggio...\n");
	while(currentSpeed > 0) {
		write(sockBrakeFd, "Freno 5", strlen("Freno 5"));
		read(sockBrakeFd, buffer, sizeof(buffer));
		currentSpeed += atoi(buffer);
		logServerEvent("FRENO 5 ---> ", true);
		sleep(1);
	}
	return (currentSpeed == 0);
}

void parkCar() {
	int sockParkfd[2];
	char bufferPark[16];
	int elapsedSeconds = 0;
	
	if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockParkfd) == -1) {
		perror("Errore nella creazione della socket tra il server e park assist");
		exit(1);
	}
	
	parkFork:
		pid_t park_pid = fork();
		
		if (park_pid < 0) {
			perror("Errore nella creazione del processo park assist!");
			exit(1);
		} else if (park_pid == 0) {
			setpgid(0, 0);
			signal(SIGTERM, sigTermParkHandler);
			int parkFD = open(sensorsSource, O_RDONLY);
			FILE *logFile = fopen(ASSIST_LOG, "a");
			
			if (parkFD != -1 && logFile != NULL) {
				int surroundCamFd = activateSurroundCameras();
				while (elapsedSeconds <= 30) {	
					sleep(1);
					read(surroundCamFd, bufferPark, sizeof(bufferPark));
					read(parkFD, bufferPark, sizeof(bufferPark));
					write(sockParkfd[1], bufferPark, strlen(bufferPark));
					logParkingEvent(logFile, bufferPark);		
					elapsedSeconds++;
				}
			} else {
				printf("Errore nell'apertura di %s o di %s\n! Surround view cameras non attivo!", sensorsSource, ASSIST_LOG);
				exit(1);
			}
		} else {
			char phaseBuf[30];
			while (elapsedSeconds <= 30) {		
				sleep(1);
				read(sockParkfd[0], bufferPark, sizeof(bufferPark));
				for (int i = 0; i < sizeof(bufferPark); i++) {
					if (bytesCheckFailure(&bufferPark[i])) {
						logServerEvent("PROCEDURA DI PARCHEGGIO INTERROTTA", false);
						logServerEvent("RIAVVIO PROCEDURA", false);
						kill(park_pid, SIGTERM);
						goto parkFork; // restart procedure
					}
				}
				snprintf(phaseBuf, sizeof(phaseBuf), "FASE -> %d/30", elapsedSeconds);
				logServerEvent(phaseBuf, false);		
				elapsedSeconds++;
			} 
			kill(park_pid, SIGTERM);
		}
}

int activateSurroundCameras() {
	int sockSurroundCamfd[2];
	char bufferSurroundCam[8];
	if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockSurroundCamfd) == -1) {
		perror("Errore nella creazione della socket tra il server e park assist");
		exit(1);
	}
	
	pid_t sorroundCam_pid = fork();
	
	if (sorroundCam_pid < 0) {
		perror("Errore nella creazione del processo surround cameras!");
		exit(1);
	} else if (sorroundCam_pid == 0) {
		int camFD = open(sensorsSource, O_RDONLY);
		FILE *logFile = fopen(CAMERAS_LOG, "a");
		if (camFD != -1 && logFile != NULL) {
			while(1)  {
				sleep(1);
				if (read(camFD, bufferSurroundCam, sizeof(bufferSurroundCam)) != sizeof(bufferSurroundCam)) {
					continue;
				} 
				write(sockSurroundCamfd[1], bufferSurroundCam, strlen(bufferSurroundCam));
				logParkingEvent(logFile, bufferSurroundCam);
			}
		} else {
			printf("Errore nell'apertura di %s o di %s\n! Surround view cameras non attivo!", sensorsSource, CAMERAS_LOG);
			exit(1);
		}
	}
	return sockSurroundCamfd[0];
}

bool bytesCheckFailure(char byte[]) {
	return (strcmp(byte, "172A") == 0 || strcmp(byte, "D693") == 0 || strcmp(byte, "BDD8") == 0 || strcmp(byte, "FAEE") == 0 || strcmp(byte, "4300") == 0 || strcmp(byte, "") == 0);
}

void sigusr1Handler(int sigNum) { // gestore arresto/pericolo
	printf("Il Server ha ricevuto un segnale di arresto/pericolo!\nArresto l'auto...\n");
	logBrakeEvent("ARRESTO AUTO");
} 

void sigusr2Handler(int sigNum) { // gestore fallimento throttle
	printf("Il Server ha ricevuto un segnale di fallimento da throttle Control!\n");
	currentSpeed = 0;
	logBrakeEvent("ARRESTO AUTO");
	logServerEvent("ARRESTO AUTO (FALLIMENTO THROTTLE CONTROL) --->", true);
	printf("Termino il programma...\n");
        signal(SIGQUIT, SIG_IGN);
	kill(0, SIGQUIT);
	logServerEvent("NAVIGAZIONE TERMINATA", false);
	exit(0);
} 

void sigTermParkHandler(int sigNum) {
	kill(0, SIGTERM);
	exit(0);
}

void logServerEvent(char event[], bool logSpeed) {
	FILE *file = fopen(MAIN_LOG_FILE, "a");
	if (file != NULL) {
		if (logSpeed) {
			fprintf(file, "%s %d km/h\n", event, currentSpeed);
		} else {
			fprintf(file, "%s\n", event);
		}
		fclose(file);
	} else {
		printf("Impossibile aprire il file di log 'throttle.log' per la registrazione.\n");
	}
}


void logSteerEvent(char eventDirection[]) {
	FILE *fileLog = fopen(STEER_LOG, "a");
	if (fileLog != NULL) {
		if (strcmp(eventDirection, "DESTRA\n") == 0 || strcmp(eventDirection, "SINISTRA\n") == 0) {
			fprintf(fileLog, "STO GIRANDO A %s", eventDirection);
		} else {
			fprintf(fileLog, "%s", eventDirection);
		}
		fclose(fileLog);
	} else {
		printf("Impossibile aprire il file di log 'steer.log' per la registrazione.\n");
	}
	
}

void logThrottleEvent() {
	time_t now;
        time(&now);
        char dataOra[100];
        strftime(dataOra, sizeof(dataOra), "%Y-%m-%d %H:%M:%S", localtime(&now));
        FILE *fileLog = fopen(THROTTLE_LOG, "a");
        
        if (fileLog != NULL) {
		fprintf(fileLog, "%s: INCREMENTO 5\n", dataOra);
		fclose(fileLog);
	} else {
		printf("Impossibile aprire il file di log 'throttle.log' per la registrazione.\n");
	}
}

void logBrakeEvent(char event[]) {
	time_t now;
        time(&now);
        char dataOra[100];
        strftime(dataOra, sizeof(dataOra), "%Y-%m-%d %H:%M:%S", localtime(&now));
	FILE *fileLog = fopen(BRAKE_LOG, "a");
	
	if (fileLog != NULL) {
		fprintf(fileLog, "%s: %s\n", dataOra, event);
		fclose(fileLog);
	} else {
		printf("Impossibile aprire il file di log 'brake.log' per la registrazione.\n");
	} 
}

void logParkingEvent(FILE* log, char bytes[]) {
	for (int i = 0; i < strlen(bytes); i++) {
		fprintf(log, "%02x ", bytes[i]);
	}
	fprintf(log, "\n");
	fflush(log);
}

int clearLogFiles() {
	return (remove(MAIN_LOG_FILE) + remove(CAMERA_LOG) + remove(RADAR_LOG) + remove(THROTTLE_LOG) +
		      remove(BRAKE_LOG) + remove(STEER_LOG) + remove(ASSIST_LOG) + remove(CAMERAS_LOG));
}
