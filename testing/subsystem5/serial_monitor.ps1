# serial_monitor.ps1 - Pico USB console that survives resets.
#
# The Pico's USB serial port disappears on every reset and comes back about
# a second later.  This script keeps retrying, so the boot messages are
# caught without restarting anything.  Everything is also saved to a log.
#
# Usage (PowerShell, from the repo root):
#   powershell -ExecutionPolicy Bypass -File testing\subsystem5\serial_monitor.ps1
#   ... -Port COM5 -Quiet   (-Quiet hides periodic status lines on screen)
# Stop with Ctrl+C.

param(
    [string]$Port = 'COM5',
    [int]$Baud = 115200,
    [switch]$Quiet,
    [string]$LogDir = (Join-Path $PSScriptRoot 'logs')
)

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
$log = Join-Path $LogDir ((Get-Date -Format 'yyyy-MM-dd_HHmmss') + '_console.txt')
Write-Host "Logging to $log  (Ctrl+C to stop)" -ForegroundColor Cyan

# Periodic lines that drown out the events (still written to the log)
$noise = '^(Buddy4: n=|\[MOTION\] state=|\[BARCODE (RAW|LEVEL|COUNT|STATUS|INFO)\]|\[MOTOR READBACK\]|\[ALL TEST\] phase=)'
$p = $null
$pending = ''
$writer = New-Object System.IO.StreamWriter($log, $true)
$writer.AutoFlush = $true

try {
    while ($true) {
        if ($null -eq $p -or -not $p.IsOpen) {
            try {
                $p = New-Object System.IO.Ports.SerialPort $Port, $Baud
                $p.DtrEnable = $true   # USB CDC only sends once DTR is set
                $p.Open()
                $msg = "=== $Port connected $(Get-Date -Format 'HH:mm:ss.fff') ==="
                Write-Host $msg -ForegroundColor Green
                $writer.WriteLine($msg)
            } catch {
                $p = $null
                Start-Sleep -Milliseconds 100
                continue
            }
        }
        try {
            $chunk = $p.ReadExisting()
        } catch {
            try { $p.Close() } catch {}
            $p = $null
            $msg = "=== $Port lost $(Get-Date -Format 'HH:mm:ss.fff') (reset?) ==="
            Write-Host $msg -ForegroundColor Yellow
            $writer.WriteLine($msg)
            continue
        }
        if ($chunk) {
            $writer.Write($chunk)
            $pending += $chunk
            $lines = $pending -split "`n"
            $pending = $lines[-1]            # keep the unfinished line
            if ($lines.Count -lt 2) { continue }
            foreach ($line in $lines[0..($lines.Count - 2)]) {
                if ($Quiet -and $line -match $noise) { continue }
                Write-Host $line.TrimEnd("`r")
            }
        }
        Start-Sleep -Milliseconds 20
    }
} finally {
    if ($p) { try { $p.Close() } catch {} }
    $writer.Close()
}
