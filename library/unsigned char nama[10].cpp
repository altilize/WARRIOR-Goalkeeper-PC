unsigned char nama[10]
unsigned char nim[10]
unsigned char nama2[10]
unsigned char nim2[10]
unsigned char umur[10]
unsigned char fakultas[10]

struct daftar {
unsigned char nama[10]
int nim 
int umur
unsigned char fakultas[10]
}

list daftar
char usbmikro =
struct tes{
    int noport;
}

serial1 tes

serial1.noport=char usbmikro

-------------------------------------------------------

struct serial {
    std::string port;
    int baudRate;
    int termios;
}

serial arduino1;
arduino1.port = "/dev/ttyACM0"
arduino1.baudRate = 9600;


serial arduino2;
arduino2.port = "/dev/ttyACM0"
arduino2.baudRate = 9600;

void init(serial* serial) {
    &serial.termios = open(,,,,)
    ...
}

void kirim(serial* serial) {
kirim data
}

void baca(serial* serial) {
    baca
}

init(&arduino1)
init(&arduino2)

-------------------------

class Serial {
    private:
        std::string port_;
        int baudRate_;

    public:
        Serial(string port, int baudRate) { // constructor
            port_ = port;
            baudRate_ = baudRate;

            cout << "halo"
        }

        ~Serial() { // deconstructor
            cout << "bye"
        }

        void init();
        void baca(int length);
        void tulis();
        //
        // void setPort(string port){
        //     port_ = port;
        // }
        // void setBaudRate(int baudrate) {
        //     baudRate_ = baudrate;
        // }
}

Serial arduino1 = Serial("/dev/ttyACM0", 9600);
// print halo
Serial arduino2 = Serial("/dev/ttyACM1", 115200);;

arduino1.port = ";"  /// tidak bisa, karena variable port private

// arduino.setPort("/dev/ttyACM0");
// arduino.setBaudRate(9600);
arduino.init(); // bisa, karena public
// arduino.baca(10); // bisa karena public
arduino.tulis(); // bisa karena public

arduino2.baca(12);

delete arduino;
// print bye
delete arduino2;
// print bye