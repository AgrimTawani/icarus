using UnrealBuildTool;
using System.IO;

public class Icarus7AirSim : ModuleRules
{
    public Icarus7AirSim(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        // AirLib headers (pulled in via AFlyingPawn) use throw/try; match
        // AirSim.Build.cs so non-editor (game) targets compile too.
        bEnableExceptions = true;
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "Engine", "AirSim"
        });
        // AirSim 1.8 predates the modern Public/Private module layout; its
        // exported AFlyingPawn and APIPCamera headers live under Source/.
        PublicIncludePaths.Add(Path.GetFullPath(
            Path.Combine(ModuleDirectory, "../../../AirSim/Source")));
    }
}
