$portInfo = Get-CimInstance Win32_SerialPort |
    Where-Object { $_.Name -like "*XDS110 Class Application/User UART*" } |
    Select-Object -First 1

if ($null -eq $portInfo) {
    Write-Error "XDS110 Application/User UART was not found."
    exit 1
}

$serial = [System.IO.Ports.SerialPort]::new(
    $portInfo.DeviceID, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.ReadTimeout = 500
$serial.DtrEnable = $false
$serial.RtsEnable = $false

try {
    $serial.Open()
    Write-Host "Reading $($portInfo.Name) at 115200 8N1. Press Ctrl+C to stop."
    while ($true) {
        $text = $serial.ReadExisting()
        if ($text.Length -ne 0) {
            [Console]::Write($text)
        }
        Start-Sleep -Milliseconds 20
    }
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
