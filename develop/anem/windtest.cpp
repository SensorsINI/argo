#include <unistd.h>
#include <cstdint>
#include <stdlib.h>
#include <wiringPiI2C.h>
#include <cerrno>
#include <sstream>
#include <iostream>
#include <wiringPi.h>
#include <cmath>
#include <cstring>
using namespace std;


/* PRIVATE FUNCTIONS */
bool checkCrc(uint8_t data[], uint8_t posInit, uint8_t checksum){
  uint8_t crc = 0xFF; //

  // calculates 8-Bit checksum with given polynomial 0x31 (x^8 + x^5 + x^4 + 1)
  for(int i = posInit; i < posInit + 2; i++) {
    crc ^= (data[i]);
    for(uint8_t bit = 8; bit > 0; --bit) {
      if(crc & 0x80) {crc = (crc << 1) ^ 0x31;}
      else           {crc = (crc << 1);}
    }
  }
  if(crc == checksum){
    return true;
  }
  else{
    cout <<"Crc does not match\n";
    return false;
  }
}

/*
brief  > checks all the crcs of the sequence received
param  > sequence vector, length of
return > bool, true if correct
 */
bool checkAllCrc(uint8_t idBuffer[]){ 
  bool result = true;
  for(uint8_t i = 0; i<(9 - 2); i = i+3){
    if(!checkCrc(idBuffer,  i, idBuffer[i+2])){
      result = false;
    }
  }
  return result;
}


int main()
{
	int err;
	int address=0x23;
	uint8_t readings[9];
	//Setup Windsensor

	wiringPiSetup();

	int fd1 = wiringPiI2CSetup (address);
	if (fd1<0) {
		cout << "Setup Error "<< endl;
		close(fd1);
		exit(1);
	}

	delay(100);
	cout << "setup device  " << address << endl;

//	err=wiringPiI2CWriteReg8(fd1, 0, 0x6); // reset entire i2c bus
//	if (err<0) {
//		cout << "reset error "<< endl;
//		close(fd1);
//		exit(1);
//	}
//
//	delay(100);
//	cout << "reset i2c complete  " << endl;

	int no_meas=0;

	err=wiringPiI2CWriteReg16(fd1, 0x3f, 0xf9); // stop previous measurement
	if(err!=0){
		cout<<"stop previous measurement Write error"<<endl;
		cout << "Error: " << strerror(errno) <<endl;
		close(fd1);
		exit(1);
	}
	delay(10);
	cout << "stopped previous continuous measurement " << endl;
	err=wiringPiI2CWriteReg16(fd1, 0x36, 0x15); // continuous measurement
	if(err!=0){
		cout<<"start measurement Write error"<<endl;
		cout << "Error: " << strerror(errno) <<endl;
		close(fd1);
		exit(1);
	}
	cout << "started continuous measurement " << endl;
	delay(100); // startup
	while (1) {		
		no_meas++;
//		err=wiringPiI2CWriteReg16(fd1, 0x36, 0x2f); // triggered measurement
//		if(err!=0){
//			cout<<"Write error writing triggered measurement"<<endl;
//			cout << "Error: " << strerror(errno) <<endl;
//			exit(1);
//		}
//		delay(50); // nothing for 45ms
		err=read(fd1,readings,9); //readings[i] = wiringPiI2CRead(fd1);
		if(err<0){
			cout<<"Measurement error:  "<<err<<endl;
			cout << "Error: " << strerror(errno) <<endl;
			close(fd1);
			exit(1);
		}
		if(!checkAllCrc(readings)){
			cout <<"CRC error"<<endl;
		}
		cout << "Measurement number "<<no_meas<<":  ";
		//for(int i=0;i<9;i++){cout<< " "<< (0xff &readings[i]);}
		int16_t dpraw=(readings[0]<<8)|readings[1];
		int16_t dpscale=((readings[6])<<8)|(readings[7]);
		float dpPascal=(float)dpraw/(float)dpscale;
		printf("diff pressure %10.4f\n",dpPascal);
		//cout <<" diff. pressure is raw="<<dpraw<<" scaleFactor="<< dpscale << " pascal="<<dpPascal;
		//cout << endl;
		delay(100);
	}
}

