const electron = require('electron')

const app = electron.app;
const BrowserWindow = electron.BrowserWindow;

const path = require('path')
const url = require('url')

let mainWindow;

function createWindow() {
    mainWindow = new BrowserWindow({width: 1360, height: 600})

    mainWindow.loadURL(url.format({
        pathname: path.join(__dirname, 'index.html'),
        protocol: 'file:',
        slashes: true
    }))

    mainWindow.webContents.openDevTools()

    mainWindow.on('closed', function () {
        mainWindow = null
    })
}

app.on('ready', createWindow)
app.on('window-all-closed', function () {
    app.quit()
})

app.on('activate', function () {
    e
    if (mainWindow === null) {
        createWindow()
    }
})
