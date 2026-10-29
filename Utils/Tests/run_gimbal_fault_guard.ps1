param([Parameter(Mandatory = $true)][string]$Compiler)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$stage = Join-Path ([IO.Path]::GetTempPath()) ('gimbal-guard-' + [guid]::NewGuid())
$encoding = New-Object System.Text.UTF8Encoding($false)

# TCC 0.9.27 treats slash/backslash aliases as different files for pragma once.
# Add conventional include guards ONLY in the temporary host-test copy.
$headers = & rg --files --glob '*.h' (Join-Path $projectRoot 'User')
foreach ($header in $headers) {
  $relative = $header.Substring($projectRoot.Length + 1)
  $destination = Join-Path $stage $relative
  [IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($destination)) | Out-Null
  $guard = 'HOST_TEST_' + ($relative -replace '[^a-zA-Z0-9]', '_').ToUpperInvariant()
  $body = [IO.File]::ReadAllText($header)
  [IO.File]::WriteAllText($destination,
    "#ifndef $guard`n#define $guard`n$body`n#endif`n", $encoding)
}
foreach ($relative in @('User/device/can.c', 'Utils/Tests/test_gimbal_fault_guard.c',
                        'Utils/Tests/stubs/can.h', 'Utils/Tests/stubs/cmsis_os2.h',
                        'Utils/Tests/stubs/stm32f4xx.h', 'Utils/Tests/stubs/math.h')) {
  $destination = Join-Path $stage $relative
  [IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($destination)) | Out-Null
  [IO.File]::Copy((Join-Path $projectRoot $relative), $destination)
}
$executable = Join-Path $stage 'test_gimbal_fault_guard.exe'
& $Compiler ('-I' + (Join-Path $stage 'Utils/Tests/stubs')) `
            ('-I' + (Join-Path $stage 'User')) -o $executable `
            (Join-Path $stage 'Utils/Tests/test_gimbal_fault_guard.c')
if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed.' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Host fault injection test failed.' }
