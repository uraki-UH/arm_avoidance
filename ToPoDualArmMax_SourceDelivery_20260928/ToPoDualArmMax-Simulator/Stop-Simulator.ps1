param([ValidateRange(1024,65535)][int]$Port=8877)
$ErrorActionPreference='Stop'
$taskApp=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot 'app'))
$taskHash=[Security.Cryptography.SHA256]::Create()
try { $taskInstance=([BitConverter]::ToString($taskHash.ComputeHash([Text.Encoding]::UTF8.GetBytes($taskApp.ToLowerInvariant())))).Replace('-','').ToLowerInvariant().Substring(0,16) } finally { $taskHash.Dispose() }
$taskHealth=Invoke-RestMethod ('http://127.0.0.1:'+$Port+'/api/health') -TimeoutSec 2
if ($taskHealth.instance -ne $taskInstance) { throw 'This port belongs to a different folder or application. Nothing was stopped.' }
$taskProcess=Get-Process -Id ([int]$taskHealth.pid) -ErrorAction Stop
$taskStarted=([DateTimeOffset]$taskProcess.StartTime).ToUnixTimeSeconds()
if ($taskProcess.ProcessName -ne 'node' -or -not $taskHealth.startedAt -or [Math]::Abs($taskStarted-[double]$taskHealth.startedAt) -gt 3) { throw 'Server process identity changed. Nothing was stopped.' }
$taskNode=(Get-Command node -ErrorAction Stop).Source
& $taskNode -e "process.kill(Number(process.argv[1]), 'SIGTERM')" ([string]$taskHealth.pid)
if ($LASTEXITCODE -ne 0) { throw 'Unable to stop the verified server process.' }
Write-Host ('Stopped simulator on port '+$Port)
