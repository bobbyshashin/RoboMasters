<template xmlns:v-bind="http://www.w3.org/1999/xhtml" xmlns:v-on="http://www.w3.org/1999/xhtml">
    <div>
        <section class="section">
            <div id="error"></div>
            <div class="columns">
                <div class="column is-11">
                    <div id="ports"></div>
                </div>
                <div class="column is-1">
                    <select class="select" id="com-selector" v-model="selectedComPort">
                        <option v-for="option in ports" v-bind:value="option">
                            {{ option }}
                        </option>
                    </select>
                    <a class="button is-primary is-fullwidth" v-on:click="!connected ? connectToRobot() : disconnectFromRobot()" v-bind:class="{ 'is-primary': !connected, 'is-danger': connected }">{{connected ? "Disconnect" : "Connect"}}</a>
                </div>
            </div>

            <br/>

            <div class="level">
                <div class="level-left">
                    <div class="level-item">

                    </div>
                </div>
            </div>

            <div class="columns">
                <div class="column is-6">
                    <div class="level">
                        <div class="level-left">
                            <div class="level-item">
                                <span class="tag is-medium">Motor State</span> <code id="motor-state">{{robot.motor}}</code>
                            </div>
                        </div>
                    </div>
                    <div id="motorPowerChart"></div>
                </div>
                <div class="column is-6">
                    <div class="level">
                        <div class="level-left">
                            <div class="level-item">
                                <code id="control-state">{{robot.control[selectedPID]}}</code>
                            </div>
                        </div>
                        <div class="level-right">
                            <div class="level-item">
                                <select class="select" id="control-system-selector" v-model="selectedPID">
                                    <option value="motor">motor</option>
                                    <option value="gimbal">gimbal</option>
                                </select>
                            </div>
                        </div>
                    </div>
                    <div id="inputSpeedChart"></div>
                </div>
            </div>
        </section>
    </div>
</template>

<script>
    import "d3";
    import "c3/c3.css";

    import _ from "lodash";
    import RobotSocket from "../RobotSocket";
    import serialport from "serialport";
    import createTable from "data-table";
    import c3 from "c3";

    let motorChart;
    export default {
        name: 'home',
        data: () => {
            return {
                robot: {
                    motor: {
                        current: 0,
                        voltage: 0,
                        power: 0,
                    },
                    control: {
                        motor: {
                            P: 0, I: 0, D: 0, output: 0,
                        },
                        gimbal: {
                            P: 0, I: 0, D: 0, target: 0, output: 0,
                        }
                    }
                },
                selectedPID: 'motor',
                selectedComPort: '',
                connected: false,
                ports: [],
            }
        },
        created() {
            const self = this;
            serialport.list((err, ports) => {
                self.ports = _.map(ports, 'comName');
                self.selectedComPort = self.ports[0];
                if (err) {
                    document.getElementById('error').textContent = err.message
                    return
                } else {
                    document.getElementById('error').textContent = ''
                }

                if (ports.length === 0) {
                    document.getElementById('error').textContent = 'No ports were discovered.'
                }

                if (ports[0]) {
                    const headers = Object.keys(ports[0])
                    const table = createTable(headers)
                    let tableHTML = ''
                    table.on('data', data => tableHTML += data)
                    table.on('end', () => document.getElementById('ports').innerHTML = tableHTML)
                    ports.forEach(port => table.write(port))
                    table.end();
                }

            })
        },
        mounted() {
            motorChart = c3.generate({
                bindto: "#motorPowerChart",
                data: {
                    x: 'x',
                    columns: [
                        ['x', '2013-01-01', '2013-01-02', '2013-01-03', '2013-01-04', '2013-01-05', '2013-01-06'],
                        ['data1', 30, 200, 100, 400, 150, 250],
                        ['data2', 130, 340, 200, 500, 250, 350]
                    ]
                },
                axis: {
                    x: {
                        type: 'timeseries',
                        tick: {
                            format: '%Y-%m-%d'
                        }
                    }
                }
            });
            motorChart.load({
                columns: [
                    ['data1', 400, 500, 450, 700, 600, 500]
                ]
            })
        },
        methods: {
            connectToRobot() {
                const processData = (data) => {
                    if (data) {
                        data = data.split("|");
                        if (data[0] && data.length === 9) {
                            data.shift();
                            console.log(data);

                            this.robot.motor.voltage = parseFloat(data[0]);
                            this.robot.motor.current = parseFloat(data[1]);
                            this.robot.motor.power = this.robot.motor.voltage * this.robot.motor.current;
                            this.robot.control.gimbal.output = parseFloat(data[2]);
                            this.robot.control.gimbal.target = parseFloat(data[3]);
                            this.robot.control.gimbal.P = parseFloat(data[4]);
                            this.robot.control.gimbal.I = parseFloat(data[5]);
                            this.robot.control.gimbal.D = parseFloat(data[6]);
                        }
                    }
                }
                RobotSocket.connectToRobot(this.selectedComPort, (connected) => this.connected = connected, processData);
            },
            disconnectFromRobot() {
                if (this.connected) {
                    RobotSocket.disconnectFromRobot();
                }
            }
        },
        components: {}
    }
</script>

<style>
    #motor-state {
        margin-left: 16px;
    }

    body {
        font-size: 18px !important;
    }

    #com-selector {
        margin-bottom: 8px;
    }
</style>
