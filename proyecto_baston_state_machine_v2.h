
// Habilitacion de debug para la impresion por el puerto serial ...
//----------------------------------------------
#define SERIAL_DEBUG_ENABLED 1

#if SERIAL_DEBUG_ENABLED
  #define DebugPrint(str)\
      {\
        Serial.println(str);\
      }
#else
  #define DebugPrint(str)
#endif

#define DebugPrintEstado(estado,evento)\
      {\
        String est = estado;\
        String evt = evento;\
        String str;\
        str = "-----------------------------------------------------";\
        DebugPrint(str);\
        str = "EST-> [" + est + "]: " + "EVT-> [" + evt + "].";\
        DebugPrint(str);\
        str = "-----------------------------------------------------";\
        DebugPrint(str);\
      }
//----------------------------------------------

//----------------------------------------------
// Estado de un sensor ...
#define ESTADO_SENSOR_OK                            108
#define ESTADO_SENSOR_ERROR                         666
//----------------------------------------------

//----------------------------------------------
// Estado de un mensaje ...
#define MENSAJE_ENVIADO_OK                          10
#define MENSAJE_ENVIADO_ERROR                       666
//----------------------------------------------


// Otras constantes ....
//----------------------------------------------
#define UMBRAL_DIFERENCIA_TIMEOUT                   50
#define UMBRAL_OBJETO_CERCANO                       20
#define UMBRAL_OBJETO_MEDIO                         80

#define MAX_CANT_SENSORES                           2
#define SENSOR_PROXIMIDAD                           0
#define SENSOR_AGARRE                               1
#define PIN_SENSOR_PROXIMIDAD_ECHO                  6
#define PIN_SENSOR_PROXIMIDAD_TRIGGER               7
#define PIN_SENSOR_AGARRE                           "A0"
#define MIN_FUERZA_ACEPTADA                         50

#define PIN_MOTOR                                   3
#define PIN_BUZZER                                  998
                           

#define byte 255
#define distCalculator 58.2
//----------------------------------------------

//----------------------------------------------
struct stSensor
{
  int  pin;
  int  estado;
  long valor_actual;
  long valor_previo;
};
stSensor sensores[MAX_CANT_SENSORES];
//----------------------------------------------

enum states          { ST_RUNNING     ,  ST_OBJECT_NEAR      ,ST_GRABBED         ,ST_ALERT_OBJECT_NEAR_TRIGGERED       , ST_ALERT_FALL_TRIGGERED} current_state;
String states_s [] = {"ST_RUNNING"  ,  "ST_OBJECT_NEAR"      ,"ST_GRABBED"         ,"ST_ALERT_OBJECT_NEAR_TRIGGERED"       , "ST_ALERT_FALL_TRIGGERED" };

enum events          { EV_OBJECT_NEAR        , EV_OBJECT_AWAY        , EV_GRABBED        , EV_NOT_GRABBED} new_event;
String events_s [] = { "EV_OBJECT_NEAR"        , "EV_OBJECT_AWAY"        , "EV_GRABBED"        , "EV_NOT_GRABBED"};

#define MAX_STATES 5
#define MAX_EVENTS 4

typedef void (*transition)();


//VERIFICAR Y DEFINIR QUE HACE CADA UNA DE LAS ACCIONES, LA UNICA DEFINIDA ES START_ALARM
  transition state_table_actions[MAX_STATES][MAX_EVENTS] =
  {
    {start_near             , none                  , start_grab        , none            } , // state ST_RUNNING
    {start_near             , stop_near             , start_grab        , none            } , // state ST_OBJECT_NEAR
    {start_grab             , stop_near             , none              , none            } , // state ST_GRABBED
    {start_near             , stop_near             , none              , start_alarm     } , // state ST_ALERT_OBJECT_NEAR_TRIGGERED
    {none                   , none                  , start_grab        , none            } , // state ST_ALERT_FALL_TRIGGERED
    //EV_OBJECT_NEAR        , EV_OBJECT_AWAY        , EV_GRABBED        , EV_NOT_GRABBED         
  };

bool timeout;
long lct;
long distancia, duracion, dutyCicle, agarre;


//----------------------------------------------
void do_init()
{
  Serial.begin(9600);
  
  pinMode(PIN_SENSOR_PROXIMIDAD_ECHO, INPUT); //echo es de entrada
  pinMode(PIN_SENSOR_PROXIMIDAD_TRIGGER, OUTPUT); //trigger es de salida
  pinMode(PIN_MOTOR, OUTPUT); //motor es de salida
//   pinMode(PIN_SENSOR_AGARRE, INPUT); //agarre es de entrada
  pinMOde(PIN_BUZZER, OUTPUT); //buzzer es de salida

  sensores[SENSOR_PROXIMIDAD].pin    = PIN_SENSOR_PROXIMIDAD_ECHO;
  sensores[SENSOR_PROXIMIDAD].estado = ESTADO_SENSOR_OK;

  sensores[SENSOR_AGARRE].pin           = PIN_SENSOR_AGARRE;
  sensores[SENSOR_AGARRE].estado        = ESTADO_SENSOR_OK;
  
  // Inicializo el evento inicial
  current_state = ST_RUNNING;
  
  timeout = false;
  lct     = millis();

  init_(); //Revisar si esto va aca
}
//----------------------------------------------


//----------------------------------------------
void leerSensorProximidad( )
{
    digitalWrite(PIN_SENSOR_PROXIMIDAD_TRIGGER, HIGH); //generacion del pulso a enviar
    digitalWrite(PIN_SENSOR_PROXIMIDAD_TRIGGER, LOW);
    duracion = pulseIn(PIN_SENSOR_PROXIMIDAD_ECHO, HIGH); //con pulseIn se espera un pulso alto en echo
    distancia = duracion / distCalculator; //calculo de distancia medida en centimetros
    if(distancia > byte){
        distancia = byte;
    }
}
//----------------------------------------------

void leerSensorAgarre( ){
    int lectura = analogRead(PIN_SENSOR_AGARRE); //Leo el pin conectado al sensor de fuerza y retorno su intensidad.

    if(lectura >= MIN_FUERZA_ACEPTADA)
    {
        agarre = 1; //TOMAR EL VALOR DE AGARRE (TRUE O FALSE) EN UNA VARIABLE GLOBAL - ESPERAR IMPLEMENTACION DE CHINO
    }
    else
    {
        agarre = 0
    }

}

//----------------------------------------------
void apagar_buzzer( )
{
  digitalWrite(PIN_BUZZER, false);
}
//----------------------------------------------

//----------------------------------------------
void encender_buzzer( )
{
  digitalWrite(PIN_BUZZER, true);
}
//----------------------------------------------

//----------------------------------------------
void actualizar_motor( )
{
  dutyCicle = byte - distancia;
  analogWrite(pinMotor, dutyCicle);
}
//----------------------------------------------

//----------------------------------------------


//----------------------------------------------

bool verificarEstadoSensorProximidad( )
{
  leerSensorProximidad();
  sensores[SENSOR_PROXIMIDAD].valor_actual = distancia;
  
  int valor_actual = sensores[SENSOR_TEMPERATURA].valor_actual;
  int valor_previo = sensores[SENSOR_TEMPERATURA].valor_previo;
  
  if( valor_actual != valor_previo )
  {
    sensores[SENSOR_PROXIMIDAD].valor_previo = valor_actual;
    
    if( valor_actual < UMBRAL_OBJETO_CERCANO)
    {
      new_event = EV_OBJECT_NEAR;
    }
    else if( (valor_actual >= UMBRAL_OBJETO_CERCANO)&& (valor_actual < UMBRAL_OBJETO_MEDIO) )
    {
      new_event = EV_OBJECT_AWAY;
    }
    if( valor_actual >= UMBRAL_OBJETO_MEDIO )
    {
      new_event = EV_OBJECT_AWAY;
    }
    
    return true;
  }
  
  return false;
}
//----------------------------------------------

//----------------------------------------------
bool verificarEstadoSensorAgarre()
{
    leerSensorAgarre();
    sensores[SENSOR_AGARRE].valor_actual = agarre;
    int valor_actual = sensores[SENSOR_AGARRE].valor_actual;
    int valor_previo = sensores[SENSOR_AGARRE].valor_previo;

    if(valor_actual != valor_previo){

        sensores[SENSOR_AGARRE].valor_previo = valor_actual;
        if(valor_actual == 1) //Quiere decir que no lo estaban agarrando y ahora si
        {
            new_event = EV_GRABBED;
        }else{
            new_event = EV_NOT_GRABBED;
        }
        return true;

    }
  return false;
}
//----------------------------------------------


//----------------------------------------------
void get_new_event( )
{
  long ct = millis();
  int  diferencia = (ct - lct);
  timeout = (diferencia > UMBRAL_DIFERENCIA_TIMEOUT)? (true):(false);

  if( timeout )
  {
    // Doy acuse de la recepcion del timeout
    timeout = false;
    lct     = ct;
    
    if( (verificarEstadoSensorProximidad() == true) || (verificarEstadoSensorAgarre() == true))
    {
      return;
    }
  }
  
  // Genero evento dummy ....
  new_event = EV_OBJECT_NEAR;
}
//----------------------------------------------

void init_()
{
  DebugPrintEstado(states_s[current_state], events_s[new_event]);
  current_state = ST_RUNNING;
}

void error()
{
}

void none()
{
}

void start_near(){
    actualizar_motor();
    //current_state = ST_?
}

void stop_near(){
    actualizar_motor();
    //current_state = ST_?
}

void start_alarm(){
    encender_buzzer();
    current_state = ST_ALERT_FALL_TRIGGERED;
}

void start_grab(){
    //DEFINIR ACCIONES
    //current_state = ST_?
}


//----------------------------------------------
void state_machine_proyecto_baston( )
{
  get_new_event();

  if( (new_event >= 0) && (new_event < MAX_EVENTS) && (current_state >= 0) && (current_state < MAX_STATES) )
  {
    if( new_event != EV_OBJECT_NEAR ) //OJO, ESTA RELACIONADO CON LO DEFINIDO CON EL EVENTO DUMMY DE LINEA 250, PROBABLEMENTE HAYA QUE CREAR UN EVENTO DUMMY QUE NO HAGA NADA
    {
      DebugPrintEstado(states_s[current_state], events_s[new_event]);
    }
    
    state_table[current_state][new_event]();
  }
  else
  {
    //DebugPrintEstado(states_s[ST_ERROR], events_s[EV_UNKNOW]); NO TENEMOS DEFINIDO ESTADO DE ERROR Y EVENTO DESCONOCIDO, PENSAR SI ES NECESARIO SINO SACAR ELSE
  }
  
  // Consumo el evento...
  new_event   = EV_CONT; //IGUAL LINEA 296
}
//----------------------------------------------


// Funciones de arduino !. 
//----------------------------------------------
void setup()
{
  do_init();
}
//----------------------------------------------

//----------------------------------------------
void loop()
{
  state_machine_proyecto_baston();
}
//----------------------------------------------
