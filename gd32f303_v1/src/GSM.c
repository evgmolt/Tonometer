#include "GSM.h"
#include "jWrite.h"
#include "main.h"
#include "jsmn.h"


void Login(char *login, char *password)
{
	unsigned int buflen= 1024;
  	char buffer[buflen];
	int err;
	struct jWriteControl jwc;
    jwOpen(buffer, buflen, JW_OBJECT, JW_PRETTY);
    jwObj_string( "app.username", login );	
    jwObj_string( "app.password", password );	
    jwEnd();
}
/*
void MeasResult(char *buffer, int id, int sys, int dia, int pulse, char *dt)
{
	unsigned int buflen = 1024;
	int ind;
	struct jWriteControl jwc;

	uint8_t string_buff[20] = { 0 };

	jwOpen(buffer, buflen, JW_ARRAY, JW_PRETTY);
	jwArr_object();
	sprintf(string_buff, "%d", id);
	jwObj_string("ownerId", string_buff);
	jwObj_string("source", "tonometer");
	jwObj_string("type", "med.blood_pressure");
	sprintf(string_buff, "%d/%d", sys, dia);
	jwObj_string("value", string_buff);
	jwObj_object("extraParams");
	jwObj_string("app.comment", "");
	jwEnd();
	sprintf(string_buff, "%d-%02d-%02dT%02d:%02d:%02d", dt[0] + 2000, dt[1], dt[2], dt[3], dt[4], dt[5]);
	jwObj_string("datetime", string_buff);
	jwEnd();

	jwArr_object();
	sprintf(string_buff, "%d", id);
	jwObj_string("ownerId", string_buff);
	jwObj_string("source", "tonometer");
	jwObj_string("type", "med.pulse");
	sprintf(string_buff, "%d", pulse);
	jwObj_string("value", string_buff);
	jwObj_object("extraParams");
	jwObj_string("app.comment", "");
	jwEnd();
	jwObj_string("datetime", "2022-10-12T09:30:05");
	jwEnd();
	jwClose();
	//добавляем { в начало и } в конец.
	ind = 0;
	while (buffer[ind] != 0) ind++;
	for (int i = 0; i <= ind; i++)
	{
		buffer[ind - i + 3] = buffer[ind - i];
	}
	buffer[0] = '{';
	buffer[1] = 0x0D;
	buffer[2] = 0x0A;
	buffer[ind + 3] = 0x0D;
	buffer[ind + 4] = 0x0A;
	buffer[ind + 5] = '}';
	buffer[ind + 6] = 0;
	printf(buffer);

}
*/