param([ValidateRange(1024,65520)][int]$Port = 8877, [switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
$taskRoot = $PSScriptRoot
$taskApp = [IO.Path]::GetFullPath((Join-Path $taskRoot 'app'))
$taskNode = (Get-Command node -ErrorAction SilentlyContinue).Source
if (-not $taskNode) { Write-Host 'Install Node.js 22 or later from https://nodejs.org/ and run START.cmd again.'; exit 1 }
$taskMajor = [int]((& $taskNode --version).TrimStart('v').Split('.')[0])
if ($taskMajor -lt 22) { Write-Host 'Node.js 22 or later is required.'; exit 1 }
$taskHash = [Security.Cryptography.SHA256]::Create()
try { $taskInstance = ([BitConverter]::ToString($taskHash.ComputeHash([Text.Encoding]::UTF8.GetBytes($taskApp.ToLowerInvariant())))).Replace('-','').ToLowerInvariant().Substring(0,16) }
finally { $taskHash.Dispose() }
$taskMutex = New-Object Threading.Mutex($false, 'Local\ToPoSourceDelivery-Launch')
$taskLocked = $false
try {
    try { $taskLocked = $taskMutex.WaitOne(30000) } catch [Threading.AbandonedMutexException] { $taskLocked = $true }
    if (-not $taskLocked) { throw 'Another launch is in progress.' }
    $taskChosen = $null
    $taskExisting = $false
    for ($taskCandidate=$Port; $taskCandidate -lt ($Port+16); $taskCandidate++) {
        $taskUrl = 'http://127.0.0.1:' + $taskCandidate
        try {
            $taskHealth = Invoke-RestMethod ($taskUrl+'/api/health') -TimeoutSec 1
            if ($taskHealth.instance -eq $taskInstance) { $taskChosen=$taskCandidate; $taskExisting=$true; break }
            continue
        } catch {}
        $taskTcp = New-Object Net.Sockets.TcpClient
        try { $taskConnect=$taskTcp.ConnectAsync('127.0.0.1',$taskCandidate); [void]$taskConnect.Wait(200); $taskBusy=$taskTcp.Connected }
        catch { $taskBusy=$false }
        finally { $taskTcp.Dispose() }
        if (-not $taskBusy) { $taskChosen=$taskCandidate; break }
    }
    if ($null -eq $taskChosen) { throw 'No available port. Run with -Port 9000, for example.' }
    $taskUrl='http://127.0.0.1:'+$taskChosen
    if (-not $taskExisting) {
        $taskRuntime=Join-Path $taskRoot 'runtime'
        [void](New-Item -ItemType Directory -Path $taskRuntime -Force)
        $taskOldPort=$env:PORT
        try {
            $env:PORT=[string]$taskChosen
            $taskProcess=Start-Process -FilePath $taskNode -ArgumentList @(('"'+(Join-Path $taskApp 'server.mjs')+'"')) -WorkingDirectory $taskRoot -WindowStyle Hidden -RedirectStandardOutput (Join-Path $taskRuntime 'server.log') -RedirectStandardError (Join-Path $taskRuntime 'server-error.log') -PassThru
        } finally { $env:PORT=$taskOldPort }
        $taskReady=$false
        for ($taskAttempt=0; $taskAttempt -lt 30; $taskAttempt++) {
            try { $taskHealth=Invoke-RestMethod ($taskUrl+'/api/health') -TimeoutSec 1; if ($taskHealth.instance -eq $taskInstance) { $taskReady=$true; break } } catch {}
            if ($taskProcess.HasExited) { break }
            Start-Sleep -Milliseconds 200
        }
        if (-not $taskReady) { throw 'Server did not start. See runtime/server-error.log.' }
    }
    Write-Host ('Simulator ready: '+$taskUrl+'/')
    if (-not $NoBrowser) { Start-Process ($taskUrl+'/') }
} catch { Write-Host $_.Exception.Message; exit 1 }
finally { if ($taskLocked) { $taskMutex.ReleaseMutex() }; $taskMutex.Dispose() }
