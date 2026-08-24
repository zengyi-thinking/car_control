[CmdletBinding()]
param(
    [string]$SuiteRoot
)

$ErrorActionPreference = 'Stop'
$repoNames = @('flight_control', 'car_control', 'camera_helper')

if([string]::IsNullOrWhiteSpace($SuiteRoot))
{
    $skillRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
    $skillsDir = Split-Path $skillRoot -Parent
    $agentsDir = Split-Path $skillsDir -Parent
    $scopeRoot = Split-Path $agentsDir -Parent
    $isInstalledLayout = ((Split-Path $skillsDir -Leaf) -eq 'skills') -and
        ((Split-Path $agentsDir -Leaf) -eq '.agents')

    if(-not $isInstalledLayout)
    {
        throw 'Cannot infer the project suite from a standalone skill source. Pass -SuiteRoot.'
    }
    if((Split-Path $scopeRoot -Leaf) -in $repoNames)
    {
        $resolvedSuiteRoot = [System.IO.Path]::GetFullPath((Split-Path $scopeRoot -Parent))
    }
    else
    {
        $resolvedSuiteRoot = [System.IO.Path]::GetFullPath($scopeRoot)
    }
}
else
{
    $resolvedSuiteRoot = [System.IO.Path]::GetFullPath($SuiteRoot)
}

if(-not (Test-Path -LiteralPath $resolvedSuiteRoot -PathType Container))
{
    throw "Suite root not found: $resolvedSuiteRoot"
}

$gitAvailable = $null -ne (Get-Command git -ErrorAction SilentlyContinue)
Write-Host "Feiyue Leiqu suite snapshot"
Write-Host "suite: $resolvedSuiteRoot"
Write-Host "generated: $([DateTime]::Now.ToString('yyyy-MM-dd HH:mm:ss'))"

foreach($repoName in $repoNames)
{
    $repoRoot = Join-Path $resolvedSuiteRoot $repoName
    Write-Host "`n[$repoName]"
    if(-not (Test-Path -LiteralPath $repoRoot -PathType Container))
    {
        Write-Host '  state: missing'
        continue
    }

    Write-Host '  state: present'
    if($gitAvailable -and (Test-Path -LiteralPath (Join-Path $repoRoot '.git')))
    {
        $branch = (& git -C $repoRoot branch --show-current 2>$null | Select-Object -First 1)
        $statusLines = @(& git -C $repoRoot status --short 2>$null)
        if([string]::IsNullOrWhiteSpace($branch))
        {
            $branch = '(detached or unknown)'
        }
        Write-Host "  branch: $branch"
        Write-Host "  worktree changes: $($statusLines.Count)"
    }
    else
    {
        Write-Host '  git: unavailable or repository metadata missing'
    }

    $iarRoot = Join-Path $repoRoot 'iar'
    if(Test-Path -LiteralPath $iarRoot -PathType Container)
    {
        $buildScripts = @(Get-ChildItem -LiteralPath $iarRoot -File -Filter '*.bat' |
            Where-Object { $_.Name -match '(?i)build|download' } |
            Sort-Object Name)
        if($buildScripts.Count -gt 0)
        {
            Write-Host "  build/download scripts: $($buildScripts.Name -join ', ')"
        }
        else
        {
            Write-Host '  build/download scripts: none discovered'
        }
    }

    foreach($relativePath in @('AGENTS.md', 'README.md', 'CONTINUITY.md', 'decisions.md',
                               'code\hardware\hw_config.h'))
    {
        if(Test-Path -LiteralPath (Join-Path $repoRoot $relativePath))
        {
            Write-Host "  entry: $relativePath"
        }
    }
}

$flightConfig = Join-Path $resolvedSuiteRoot 'flight_control\code\hardware\hw_config.h'
if(Test-Path -LiteralPath $flightConfig)
{
    $configPattern = '^\s*#define\s+(HW_CONTROL_PERIOD_US|HW_VISION_TRACK_ENABLE|HW_FLOW_DUAL_ENABLE|HW_MISSION_TIMED_YAW_ENABLE|HW_MISSION_AUTO_LAND_ENABLE)\b'
    $matches = @(Select-String -LiteralPath $flightConfig -Pattern $configPattern)
    if($matches.Count -gt 0)
    {
        Write-Host "`n[flight_control configuration evidence]"
        foreach($match in $matches)
        {
            Write-Host "  $($match.LineNumber): $($match.Line.Trim())"
        }
    }
}

Write-Host "`nThis snapshot is discovery evidence only. Read live source, current state, decisions and protocol endpoints before changing behavior."
