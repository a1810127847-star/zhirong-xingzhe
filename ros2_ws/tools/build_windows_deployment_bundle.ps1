[CmdletBinding()]
param()

Set-StrictMode -Version 3.0
$ErrorActionPreference = "Stop"

$ProjectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\.."))
$ArtifactRoot = Join-Path $ProjectRoot "artifacts\deployment"
$StageRoot = Join-Path $ArtifactRoot "智融行者验收一键部署"
$ArchivePath = Join-Path $ArtifactRoot "智融行者验收一键部署.zip"

if (-not $StageRoot.StartsWith($ArtifactRoot, [StringComparison]::OrdinalIgnoreCase)) {
    throw "拒绝清理意外的暂存目录：$StageRoot"
}

New-Item -ItemType Directory -Force -Path $ArtifactRoot | Out-Null
if (Test-Path -LiteralPath $StageRoot) {
    Remove-Item -LiteralPath $StageRoot -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $StageRoot | Out-Null

Copy-Item -LiteralPath (Join-Path $ProjectRoot "一键部署并打开验收.cmd") -Destination $StageRoot
Copy-Item -LiteralPath (Join-Path $PSScriptRoot "windows_bootstrap.ps1") -Destination $StageRoot
Copy-Item -LiteralPath (Join-Path $ProjectRoot "给验收人员的部署说明.txt") -Destination (Join-Path $StageRoot "使用说明.txt")

if (Test-Path -LiteralPath $ArchivePath) {
    Remove-Item -LiteralPath $ArchivePath -Force
}
Compress-Archive -Path (Join-Path $StageRoot "*") -DestinationPath $ArchivePath -CompressionLevel Optimal

Write-Output "DEPLOYMENT_BUNDLE_OK=$ArchivePath"
