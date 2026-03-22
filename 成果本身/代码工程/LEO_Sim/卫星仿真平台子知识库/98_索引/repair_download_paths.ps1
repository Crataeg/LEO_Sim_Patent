$ErrorActionPreference = "Stop"

$targetRoot = Split-Path -Parent $PSScriptRoot
$projectRoot = Split-Path -Parent $targetRoot
$paperRoot = Get-ChildItem -Path $targetRoot -Directory | Where-Object { $_.Name -like "03_*" } | Select-Object -First 1
if (-not $paperRoot) {
    throw "Cannot locate 03_* paper directory in target vault."
}
$targetPdf = Join-Path $paperRoot.FullName "PDF"
New-Item -ItemType Directory -Force -Path $targetPdf | Out-Null

$paperFiles = Get-ChildItem -Path $projectRoot -Recurse -File | Where-Object {
    $_.Name -match '^(0[1-9]|10)_.+\.pdf$' -and $_.DirectoryName -ne $targetPdf
}
foreach ($file in $paperFiles) {
    Move-Item -Force $file.FullName (Join-Path $targetPdf $file.Name)
}

$jsonFiles = Get-ChildItem -Path $projectRoot -Recurse -File -Filter "download_results.json" | Where-Object {
    $_.DirectoryName -ne $targetPdf
}
foreach ($file in $jsonFiles) {
    Move-Item -Force $file.FullName (Join-Path $targetPdf $file.Name)
}

$orphanDirs = Get-ChildItem -Path $projectRoot -Directory | Where-Object {
    $_.FullName -ne $targetRoot -and (Get-ChildItem -Path $_.FullName -Recurse -File -ErrorAction SilentlyContinue | Where-Object {
        $_.Name -match '^(0[1-9]|10)_.+\.pdf$'
    }).Count -eq 0 -and $_.Name -like "鍗*"
}
foreach ($dir in $orphanDirs) {
    Remove-Item -Recurse -Force $dir.FullName
}

Get-ChildItem -Path $targetPdf -File | Select-Object Name, Length | Sort-Object Name | Format-Table -AutoSize
