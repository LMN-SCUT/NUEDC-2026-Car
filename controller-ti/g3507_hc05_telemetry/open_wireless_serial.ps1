param(
    [Parameter(Mandatory = $true)]
    [string]$Port
)

$serial = [System.IO.Ports.SerialPort]::new(
    $Port, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.ReadTimeout = 500
$serial.DtrEnable = $false
$serial.RtsEnable = $false

try {
    $serial.Open()
    Write-Host "Reading wireless telemetry from $Port at 115200 8N1. Press Ctrl+C to stop."
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
