/**
 * Created by Dranithix on 2/16/2017.
 */
import SerialPort from "serialport";

class RobotSocket {
    socket;
    connected = false;

    connectToRobot(portId, lifecycleCallback, dataCallback) {
        const self = this;

        if (portId) {
            self.socket = new SerialPort(portId, {baudRate: 115200, parser: SerialPort.parsers.raw});
            self.socket.on('open', () => {
                self.connected = true;
                lifecycleCallback(self.connected);
            });
            self.socket.on('data', (data) => {
                if (data) dataCallback(data.toString());
            })
            self.socket.on('close', () => {
                self.connected = false;
                lifecycleCallback(self.connected);
            })
        }
    }

    disconnectFromRobot() {
        const self = this;
        if (self.socket && self.connected) {
            self.socket.close();
        }
    }
}

let instance = new RobotSocket();
export default instance;