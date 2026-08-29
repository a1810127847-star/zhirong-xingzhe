Option Explicit

Dim shell, fso, rootDir, panelScript, command, probeResult
Set shell = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")

rootDir = fso.GetParentFolderName(WScript.ScriptFullName)
panelScript = fso.BuildPath(rootDir, "ros2_ws\tools\acceptance_panel.py")
probeResult = shell.Run("cmd.exe /c where pyw.exe >nul 2>&1", 0, True)
If probeResult = 0 Then
    command = "pyw.exe -3 " & Chr(34) & panelScript & Chr(34)
Else
    probeResult = shell.Run("cmd.exe /c where pythonw.exe >nul 2>&1", 0, True)
    If probeResult <> 0 Then
        MsgBox "Windows Python 3 was not found. Install Python 3 with tkinter first.", 16, "Acceptance panel"
        WScript.Quit 1
    End If
    command = "pythonw.exe " & Chr(34) & panelScript & Chr(34)
End If
shell.Run command, 0, False
