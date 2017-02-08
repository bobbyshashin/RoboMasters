import serialport from "serialport";
import _ from "lodash";
import createTable from "data-table";

serialport.list((err, ports) => {
    console.log('ports', ports);
    if (err) {
        document.getElementById('error').textContent = err.message
        return
    } else {
        document.getElementById('error').textContent = ''
    }

    if (ports.length === 0) {
        document.getElementById('error').textContent = 'No ports discovered'
    }

    const headers = Object.keys(ports[0])
    const table = createTable(headers)
    let tableHTML = ''
    table.on('data', data => tableHTML += data)
    table.on('end', () => document.getElementById('ports').innerHTML = tableHTML)
    ports.forEach(port => table.write(port))
    table.end();
})
