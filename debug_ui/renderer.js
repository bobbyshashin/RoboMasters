import serialport from "serialport";
import createTable from "data-table";
import c3 from "c3";

const createCharts = () => {
    const timeseriesChart = c3.generate({
        bindto: '#motorPowerChart',
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
}

serialport.list((err, ports) => {
    console.log('ports', ports);
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

    createCharts();
})
