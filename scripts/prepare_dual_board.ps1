$ErrorActionPreference = 'Stop'
$repoDir = Split-Path -Parent $PSScriptRoot
$sourceProject = Join-Path $repoDir 'MDK-ARM\FINAL-2026.uvprojx'
$outputProject = Join-Path $repoDir 'MDK-ARM\dual_board.uvprojx'
[xml]$project = Get-Content -LiteralPath $sourceProject -Raw
$targets = $project.Project.Targets
$template = $targets.Target.CloneNode($true)
$targets.RemoveAll()
foreach ($role in @(@{ Name = 'UPPER'; Value = 1 }, @{ Name = 'LOWER'; Value = 2 })) {
    $target = $template.CloneNode($true)
    $target.TargetName = $role.Name
    $common = $target.TargetOption.TargetCommonOption
    $common.OutputDirectory = "build\$($role.Name)\"
    $common.OutputName = $role.Name
    $common.ListingPath = "build\$($role.Name)\"
    $common.CreateHexFile = '1'
    $common.BrowseInformation = '0'
    $compile = $target.TargetOption.TargetArmAds.Cads.VariousControls
    $defines = $compile.Define -replace ',?BOARD_ROLE=[^,]+', ''
    $compile.Define = "$defines,BOARD_ROLE=$($role.Value)"
    $group = $project.CreateElement('Group')
    $name = $project.CreateElement('GroupName'); $name.InnerText = 'Dual Board'
    [void]$group.AppendChild($name)
    $files = $project.CreateElement('Files')
    $file = $project.CreateElement('File')
    foreach ($pair in @(@('FileName','board_link.c'), @('FileType','1'), @('FilePath','..\User\board_link.c'))) {
        $node = $project.CreateElement($pair[0]); $node.InnerText = $pair[1]
        [void]$file.AppendChild($node)
    }
    [void]$files.AppendChild($file); [void]$group.AppendChild($files)
    [void]$target.Groups.AppendChild($group)
    [void]$targets.AppendChild($target)
    [void](New-Item -ItemType Directory -Path (Join-Path $repoDir "MDK-ARM\build\$($role.Name)") -Force)
}
# Mechanical XML generation keeps local compiler/debug settings from the source project.
$settings = [System.Xml.XmlWriterSettings]::new()
$settings.Indent = $true; $settings.Encoding = [System.Text.UTF8Encoding]::new($false)
$writer = [System.Xml.XmlWriter]::Create($outputProject, $settings)
try { $project.Save($writer) } finally { $writer.Dispose() }
Write-Output $outputProject
