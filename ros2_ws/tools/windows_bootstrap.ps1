[CmdletBinding()]
param(
    [switch]$Resume,
    [switch]$ValidateOnly,
    [string]$TargetRoot = ""
)

Set-StrictMode -Version 3.0
$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"

$RepoUrl = "https://github.com/a1810127847-star/zhirong-xingzhe.git"
$Branch = "master"
$Distro = "Ubuntu-22.04"
$LinuxUser = "zhirong"
$Workspace = '$HOME/zhirong_xingzhe_ws'
$StateRoot = Join-Path $env:LOCALAPPDATA "ZhirongXingzhe"
$StableScript = Join-Path $StateRoot "windows_bootstrap.ps1"
$LogPath = Join-Path $StateRoot "bootstrap.log"
$FreshDistroMarker = Join-Path $StateRoot "fresh-ubuntu-22.04.marker"
$RunOncePath = "HKCU:\Software\Microsoft\Windows\CurrentVersion\RunOnce"
$RunOnceName = "ZhirongXingzheBootstrap"

if ([string]::IsNullOrWhiteSpace($TargetRoot)) {
    $documents = [Environment]::GetFolderPath("MyDocuments")
    if ([string]::IsNullOrWhiteSpace($documents)) {
        $documents = Join-Path $env:USERPROFILE "Documents"
    }
    $TargetRoot = Join-Path $documents "ZhirongXingzhe"
}
$TargetRoot = [IO.Path]::GetFullPath($TargetRoot)

function Test-IsAdministrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Quote-PowerShellArgument([string]$Value) {
    return '"' + $Value + '"'
}

function Start-ElevatedCopy {
    $argumentLine = "-NoProfile -ExecutionPolicy Bypass -File " +
        (Quote-PowerShellArgument $PSCommandPath) +
        " -TargetRoot " + (Quote-PowerShellArgument $TargetRoot)
    if ($Resume) {
        $argumentLine += " -Resume"
    }
    Start-Process -FilePath "powershell.exe" -Verb RunAs -ArgumentList $argumentLine
}

if ((-not $ValidateOnly) -and (-not (Test-IsAdministrator))) {
    Write-Host "需要管理员权限来安装 WSL 和系统组件，正在请求 UAC 授权..." -ForegroundColor Yellow
    Start-ElevatedCopy
    exit 0
}

if (-not $ValidateOnly) {
    New-Item -ItemType Directory -Force -Path $StateRoot | Out-Null
    try {
        Copy-Item -LiteralPath $PSCommandPath -Destination $StableScript -Force
    } catch {
        Write-Warning "无法刷新续装脚本副本：$($_.Exception.Message)"
    }

    try {
        Start-Transcript -Path $LogPath -Append -Force | Out-Null
    } catch {
        Write-Warning "无法启用部署日志：$($_.Exception.Message)"
    }
}

function Write-Step([string]$Message) {
    Write-Host ""
    Write-Host "==> $Message" -ForegroundColor Cyan
}

function Refresh-ProcessPath {
    $machinePath = [Environment]::GetEnvironmentVariable("Path", "Machine")
    $userPath = [Environment]::GetEnvironmentVariable("Path", "User")
    $env:Path = "$machinePath;$userPath"
}

function Find-Executable([string[]]$Names, [string[]]$KnownPaths = @()) {
    foreach ($name in $Names) {
        $command = Get-Command $name -ErrorAction SilentlyContinue
        if ($null -ne $command) {
            return $command.Source
        }
    }
    foreach ($path in $KnownPaths) {
        if (Test-Path -LiteralPath $path -PathType Leaf) {
            return $path
        }
    }
    return $null
}

function Invoke-External(
    [string]$FilePath,
    [string[]]$Arguments,
    [switch]$AllowFailure
) {
    Write-Host ("    " + $FilePath + " " + ($Arguments -join " ")) -ForegroundColor DarkGray
    & $FilePath @Arguments 2>&1 | ForEach-Object { Write-Host "$_" }
    $exitCode = $LASTEXITCODE
    if (($exitCode -ne 0) -and (-not $AllowFailure)) {
        throw "命令失败（退出码 $exitCode）：$FilePath"
    }
    return $exitCode
}

function Require-WinGet {
    $winget = Find-Executable -Names @("winget.exe") -KnownPaths @(
        (Join-Path $env:LOCALAPPDATA "Microsoft\WindowsApps\winget.exe")
    )
    if ($null -eq $winget) {
        throw "未找到 WinGet。请先从 Microsoft Store 安装应用安装程序 (App Installer)，然后重新双击部署入口。"
    }
    return $winget
}

function Install-WinGetPackage([string]$PackageId, [string]$DisplayName) {
    $winget = Require-WinGet
    Write-Step "安装 $DisplayName"
    Invoke-External -FilePath $winget -Arguments @(
        "install", "--id", $PackageId, "--exact", "--source", "winget", "--silent",
        "--accept-package-agreements", "--accept-source-agreements"
    ) | Out-Null
    Refresh-ProcessPath
}

function Get-GitPath {
    return Find-Executable -Names @("git.exe") -KnownPaths @(
        (Join-Path $env:ProgramFiles "Git\cmd\git.exe"),
        (Join-Path ${env:ProgramFiles(x86)} "Git\cmd\git.exe")
    )
}

function Get-PythonLauncher {
    $launcher = Find-Executable -Names @("py.exe") -KnownPaths @(
        (Join-Path $env:WINDIR "py.exe"),
        (Join-Path $env:LOCALAPPDATA "Programs\Python\Python312\python.exe"),
        (Join-Path $env:ProgramFiles "Python312\python.exe")
    )
    if ($null -ne $launcher) {
        return $launcher
    }
    $pythonCommand = Get-Command "python.exe" -ErrorAction SilentlyContinue
    if (($null -ne $pythonCommand) -and ($pythonCommand.Source -notmatch '\\WindowsApps\\python\.exe$')) {
        return $pythonCommand.Source
    }
    return $launcher
}

function Get-PythonWindowedLauncher {
    return Find-Executable -Names @("pyw.exe", "pythonw.exe") -KnownPaths @(
        (Join-Path $env:WINDIR "pyw.exe"),
        (Join-Path $env:LOCALAPPDATA "Programs\Python\Python312\pythonw.exe"),
        (Join-Path $env:ProgramFiles "Python312\pythonw.exe")
    )
}

function Test-PythonTk([string]$PythonPath) {
    if ($null -eq $PythonPath) {
        return $false
    }
    if ([IO.Path]::GetFileName($PythonPath).ToLowerInvariant() -eq "py.exe") {
        & $PythonPath -3 -c "import sys, tkinter; assert sys.version_info >= (3, 9)" 2>$null
    } else {
        & $PythonPath -c "import sys, tkinter; assert sys.version_info >= (3, 9)" 2>$null
    }
    return ($LASTEXITCODE -eq 0)
}

function Get-InstalledDistros {
    $output = & wsl.exe --list --quiet 2>$null
    if ($LASTEXITCODE -ne 0) {
        return @()
    }
    return @($output | ForEach-Object { ("$_" -replace "`0", "").Trim() } | Where-Object { $_ })
}

function Test-WslRebootPending {
    if (Test-Path -LiteralPath "HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Component Based Servicing\RebootPending") {
        return $true
    }
    foreach ($featureName in @("VirtualMachinePlatform", "Microsoft-Windows-Subsystem-Linux")) {
        try {
            $feature = Get-WindowsOptionalFeature -Online -FeatureName $featureName
            if ($feature.State -eq "EnablePending") {
                return $true
            }
        } catch {
            continue
        }
    }
    return $false
}

function Register-Resume {
    New-Item -Path $RunOncePath -Force | Out-Null
    $command = 'powershell.exe -NoProfile -ExecutionPolicy Bypass -File "' +
        $StableScript + '" -Resume -TargetRoot "' + $TargetRoot + '"'
    New-ItemProperty -Path $RunOncePath -Name $RunOnceName -Value $command -PropertyType String -Force | Out-Null
}

function Clear-Resume {
    Remove-ItemProperty -Path $RunOncePath -Name $RunOnceName -ErrorAction SilentlyContinue
}

function Request-RebootAndExit {
    Register-Resume
    Add-Type -AssemblyName System.Windows.Forms
    $result = [System.Windows.Forms.MessageBox]::Show(
        "WSL/Ubuntu 已进入待重启状态。重启后安装器会自动继续。`r`n`r`n是否现在重启电脑？",
        "智融行者一键部署",
        [System.Windows.Forms.MessageBoxButtons]::YesNo,
        [System.Windows.Forms.MessageBoxIcon]::Information
    )
    if ($result -eq [System.Windows.Forms.DialogResult]::Yes) {
        Write-Host "电脑即将重启，部署会在登录后自动继续。" -ForegroundColor Yellow
        Stop-Transcript -ErrorAction SilentlyContinue | Out-Null
        Restart-Computer -Force
    }
    Write-Host "请稍后手动重启；登录 Windows 后部署会自动继续。" -ForegroundColor Yellow
    Stop-Transcript -ErrorAction SilentlyContinue | Out-Null
    exit 3010
}

function Convert-ToWslPath([string]$WindowsPath) {
    $full = [IO.Path]::GetFullPath($WindowsPath)
    if ($full -notmatch '^([A-Za-z]):\\(.*)$') {
        throw "暂不支持该 Windows 路径：$full"
    }
    $drive = $Matches[1].ToLowerInvariant()
    $rest = $Matches[2].Replace('\', '/')
    return "/mnt/$drive/$rest"
}

function Invoke-Wsl([string[]]$Arguments, [switch]$AllowFailure) {
    $allArguments = @("-d", $Distro) + $Arguments
    return Invoke-External -FilePath "wsl.exe" -Arguments $allArguments -AllowFailure:$AllowFailure
}

function Initialize-FreshDistro {
    if (-not (Test-Path -LiteralPath $FreshDistroMarker)) {
        return
    }
    Write-Step "初始化 Ubuntu 用户"
    $userSetup = @(
        "set -e",
        "if ! id -u $LinuxUser >/dev/null 2>&1; then useradd -m -s /bin/bash $LinuxUser; fi",
        "printf '[boot]\\nsystemd=true\\n[user]\\ndefault=$LinuxUser\\n' > /etc/wsl.conf"
    ) -join "; "
    Invoke-Wsl -Arguments @("-u", "root", "--", "bash", "-lc", $userSetup) | Out-Null
    Invoke-External -FilePath "wsl.exe" -Arguments @("--terminate", $Distro) -AllowFailure | Out-Null
    Start-Sleep -Seconds 2
    Remove-Item -LiteralPath $FreshDistroMarker -Force
}

function Ensure-SourceRepository([string]$GitPath) {
    Write-Step "Clone 或安全更新项目源码"
    $gitDirectory = Join-Path $TargetRoot ".git"
    if (Test-Path -LiteralPath $gitDirectory -PathType Container) {
        $originUrl = (& $GitPath -C $TargetRoot remote get-url origin).Trim()
        if ($LASTEXITCODE -ne 0) {
            throw "现有源码目录没有可读取的 origin：$TargetRoot"
        }
        $expectedOrigin = $RepoUrl.TrimEnd('/').ToLowerInvariant()
        $actualOrigin = $originUrl.TrimEnd('/').ToLowerInvariant()
        if ($expectedOrigin.EndsWith(".git")) {
            $expectedOrigin = $expectedOrigin.Substring(0, $expectedOrigin.Length - 4)
        }
        if ($actualOrigin.EndsWith(".git")) {
            $actualOrigin = $actualOrigin.Substring(0, $actualOrigin.Length - 4)
        }
        if ($actualOrigin -ne $expectedOrigin) {
            throw "目标目录属于其他 Git 仓库，已拒绝更新：$originUrl"
        }
        $dirty = & $GitPath -C $TargetRoot status --porcelain
        if ($LASTEXITCODE -ne 0) {
            throw "无法读取现有源码目录的 Git 状态。"
        }
        if ($dirty) {
            throw "目标源码目录存在未提交修改，为避免覆盖已停止自动更新：$TargetRoot"
        }
        Invoke-External -FilePath $GitPath -Arguments @(
            "-c", "http.sslBackend=openssl", "-C", $TargetRoot,
            "fetch", "--prune", "origin", $Branch
        ) | Out-Null
        Invoke-External -FilePath $GitPath -Arguments @(
            "-C", $TargetRoot, "merge", "--ff-only", "origin/$Branch"
        ) | Out-Null
        return
    }
    if (Test-Path -LiteralPath $TargetRoot) {
        $entries = @(Get-ChildItem -LiteralPath $TargetRoot -Force -ErrorAction SilentlyContinue)
        if ($entries.Count -gt 0) {
            throw "Clone 目标目录非空且不是 Git 仓库：$TargetRoot"
        }
    } else {
        New-Item -ItemType Directory -Force -Path (Split-Path -Parent $TargetRoot) | Out-Null
    }
    Invoke-External -FilePath $GitPath -Arguments @(
        "-c", "http.sslBackend=openssl", "clone", "--branch", $Branch,
        "--single-branch", "--", $RepoUrl, $TargetRoot
    ) | Out-Null
}

function Write-PanelConfig {
    $configPath = Join-Path $TargetRoot ".acceptance_panel.local.json"
    $config = [ordered]@{
        wsl_distro = $Distro
        wsl_workspace = $Workspace
        repo_url = $RepoUrl
        branch = $Branch
        source_root = $TargetRoot
    }
    $json = ($config | ConvertTo-Json) + [Environment]::NewLine
    [IO.File]::WriteAllText($configPath, $json, (New-Object Text.UTF8Encoding($false)))
}

function Launch-AcceptancePanel([string]$PythonWindowed) {
    $panel = Join-Path $TargetRoot "ros2_ws\tools\acceptance_panel.py"
    if (-not (Test-Path -LiteralPath $panel -PathType Leaf)) {
        throw "验收面板不存在：$panel"
    }
    Write-Step "启动验收面板和完整仿真"
    if ([IO.Path]::GetFileName($PythonWindowed).ToLowerInvariant() -eq "pyw.exe") {
        Start-Process -FilePath $PythonWindowed -ArgumentList @("-3", ('"' + $panel + '"'), "--auto-start") -WorkingDirectory $TargetRoot
    } else {
        Start-Process -FilePath $PythonWindowed -ArgumentList @(('"' + $panel + '"'), "--auto-start") -WorkingDirectory $TargetRoot
    }
}

if ($ValidateOnly) {
    $projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\.."))
    if (Test-Path -LiteralPath (Join-Path $PSScriptRoot "bootstrap_machine.sh") -PathType Leaf) {
        $mode = "repository"
        $required = @(
            (Join-Path $projectRoot "一键部署并打开验收.cmd"),
            (Join-Path $projectRoot "给验收人员的部署说明.txt"),
            (Join-Path $PSScriptRoot "bootstrap_machine.sh"),
            (Join-Path $PSScriptRoot "setup_workspace.sh"),
            (Join-Path $PSScriptRoot "acceptance_panel.py")
        )
    } else {
        $mode = "deployment_bundle"
        $required = @(
            (Join-Path $PSScriptRoot "一键部署并打开验收.cmd"),
            (Join-Path $PSScriptRoot "使用说明.txt")
        )
    }
    $missing = @($required | Where-Object { -not (Test-Path -LiteralPath $_ -PathType Leaf) })
    [ordered]@{
        validation = if ($missing.Count -eq 0) { "OK" } else { "FAILED" }
        mode = $mode
        repo_url = $RepoUrl
        branch = $Branch
        distro = $Distro
        target_root = $TargetRoot
        missing = $missing
    } | ConvertTo-Json -Depth 3
    if ($missing.Count -gt 0) {
        exit 1
    }
    exit 0
}

try {
    Write-Host "智融行者 Windows 一键部署" -ForegroundColor Green
    Write-Host "源码目录：$TargetRoot"
    Write-Host "部署日志：$LogPath"

    $build = [Environment]::OSVersion.Version.Build
    if ($build -lt 19041) {
        throw "Windows 版本过低。WSL 自动安装要求 Windows 10 2004（Build 19041）或更高版本，推荐 Windows 11。"
    }
    if (-not [Environment]::Is64BitOperatingSystem) {
        throw "当前项目只支持 64 位 Windows/amd64。"
    }

    Refresh-ProcessPath
    $git = Get-GitPath
    if ($null -eq $git) {
        Install-WinGetPackage -PackageId "Git.Git" -DisplayName "Git"
        $git = Get-GitPath
    }
    if ($null -eq $git) {
        throw "Git 安装完成后仍未找到 git.exe，请重启 Windows 后重新双击部署入口。"
    }

    $python = Get-PythonLauncher
    $pythonWindowed = Get-PythonWindowedLauncher
    if ((-not (Test-PythonTk $python)) -or ($null -eq $pythonWindowed)) {
        Install-WinGetPackage -PackageId "Python.Python.3.12" -DisplayName "Python 3.12（含 Tk 图形界面）"
        $python = Get-PythonLauncher
        $pythonWindowed = Get-PythonWindowedLauncher
    }
    if (-not (Test-PythonTk $python)) {
        throw "Python 安装完成后仍无法导入 tkinter，请重启 Windows 后重新双击部署入口。"
    }
    if ($null -eq $pythonWindowed) {
        throw "未找到 pyw.exe 或 pythonw.exe。"
    }

    Ensure-SourceRepository -GitPath $git
    Write-PanelConfig

    $distros = Get-InstalledDistros
    if ($distros -notcontains $Distro) {
        Write-Step "安装 WSL2 和 Ubuntu 22.04"
        Set-Content -LiteralPath $FreshDistroMarker -Value (Get-Date).ToString("o") -Encoding ASCII
        Register-Resume
        $exitCode = Invoke-External -FilePath "wsl.exe" -Arguments @(
            "--install", "--distribution", $Distro, "--no-launch"
        ) -AllowFailure
        if ($exitCode -ne 0) {
            if (($exitCode -eq 3010) -or ($exitCode -eq 1641) -or (Test-WslRebootPending)) {
                Request-RebootAndExit
            }
            Write-Warning "Microsoft Store 安装路径失败，改用官方 web-download 重试。"
            Invoke-External -FilePath "wsl.exe" -Arguments @(
                "--install", "--distribution", $Distro, "--no-launch", "--web-download"
            ) | Out-Null
        }
    }

    $probeCode = Invoke-Wsl -Arguments @("-u", "root", "--", "bash", "-lc", "echo WSL_BOOT_OK") -AllowFailure
    if ($probeCode -ne 0) {
        Request-RebootAndExit
    }

    Write-Step "确认 Ubuntu 使用 WSL2"
    Invoke-External -FilePath "wsl.exe" -Arguments @("--set-default-version", "2") | Out-Null
    Invoke-External -FilePath "wsl.exe" -Arguments @("--set-version", $Distro, "2") | Out-Null

    Initialize-FreshDistro

    $repoWsl = Convert-ToWslPath $TargetRoot
    $bootstrapWsl = "$repoWsl/ros2_ws/tools/bootstrap_machine.sh"
    $sourceWsl = "$repoWsl/ros2_ws/src"
    $setupWsl = "$repoWsl/ros2_ws/tools/setup_workspace.sh"

    Write-Step "安装 ROS2 Humble 基础环境（可能需要 10–30 分钟）"
    Invoke-Wsl -Arguments @("-u", "root", "--", "bash", $bootstrapWsl, "--mode", "system") | Out-Null

    Write-Step "安装 Gazebo、Nav2、SLAM 与项目依赖"
    Invoke-Wsl -Arguments @(
        "-u", "root", "--", "bash", $bootstrapWsl,
        "--mode", "dependencies", "--source", $sourceWsl
    ) | Out-Null

    Write-Step "构建 ROS2 工作空间"
    Invoke-Wsl -Arguments @("--", "bash", $setupWsl) | Out-Null

    $panelScript = Join-Path $TargetRoot "ros2_ws\tools\acceptance_panel.py"
    if ([IO.Path]::GetFileName($python).ToLowerInvariant() -eq "py.exe") {
        & $python -3 $panelScript --self-test
    } else {
        & $python $panelScript --self-test
    }
    if ($LASTEXITCODE -ne 0) {
        throw "验收面板自检失败。"
    }

    Clear-Resume
    Launch-AcceptancePanel -PythonWindowed $pythonWindowed
    Write-Host ""
    Write-Host "部署完成。验收面板正在打开，Gazebo、RViz 和 Nav2 将自动启动。" -ForegroundColor Green
    Write-Host "后续可双击源码目录中的 [打开项目验收面板.cmd]。"
    Stop-Transcript -ErrorAction SilentlyContinue | Out-Null
    exit 0
} catch {
    Clear-Resume
    Write-Host ""
    Write-Host "部署失败：$($_.Exception.Message)" -ForegroundColor Red
    Write-Host "日志已保存到：$LogPath" -ForegroundColor Yellow
    Write-Host "修复网络或系统问题后，重新双击部署入口即可从已有进度继续。"
    Stop-Transcript -ErrorAction SilentlyContinue | Out-Null
    exit 1
}
