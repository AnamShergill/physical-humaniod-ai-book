---
title: High-Fidelity Rendering & Lighting
description: Understanding advanced rendering techniques and lighting for photorealistic robot visualization
sidebar_label: Lesson 6.2 - High-Fidelity Rendering & Lighting
---

# High-Fidelity Rendering & Lighting

## Learning Objectives

After completing this lesson, you will be able to:
- Implement advanced rendering techniques for photorealistic robot visualization
- Configure realistic lighting systems with shadows and reflections
- Use post-processing effects to enhance visual quality
- Optimize rendering performance for real-time applications

## Introduction

High-fidelity rendering and lighting are crucial for creating photorealistic visualizations of humanoid robots in Unity. Advanced rendering techniques combined with realistic lighting systems create compelling and believable visual experiences that accurately represent robot appearance, materials, and environmental interactions. Understanding these concepts is essential for producing professional-quality visualizations that effectively communicate robot capabilities and behaviors.

## Core Concepts

### Physically-Based Rendering (PBR)
PBR is a rendering approach that simulates realistic lighting interactions with materials based on physical properties like metallicness, roughness, and albedo, resulting in more realistic appearances.

### Global Illumination
Global illumination systems simulate how light bounces around the environment, creating realistic indirect lighting, color bleeding, and ambient occlusion effects.

### Post-Processing Effects
Post-processing effects are applied after the initial rendering pass to enhance visual quality with bloom, color grading, depth of field, and other cinematic effects.

## Mental Models

### Realism vs Performance Balance
Thinking about the trade-offs between visual fidelity and rendering performance, understanding when to prioritize realistic appearance versus smooth real-time performance.

### Material Properties Understanding
Grasping how different material properties (metallicness, roughness, normal maps) affect light reflection and appearance in the rendered scene.

## Code Examples

### Example 1: Advanced Material Setup for Robot Parts
Creating realistic materials for robot components:

```csharp
using UnityEngine;
using System.Collections;

public class RobotMaterialManager : MonoBehaviour
{
    [Header("Material Presets")]
    public Material metalMaterial;
    public Material plasticMaterial;
    public Material rubberMaterial;
    public Material glassMaterial;

    [Header("Surface Properties")]
    public Texture2D normalMap;
    public Texture2D metallicMap;
    public Texture2D roughnessMap;
    public Texture2D aoMap; // Ambient Occlusion

    [Header("Dynamic Properties")]
    public float baseMetallic = 0.7f;
    public float baseRoughness = 0.3f;
    public Color baseColor = Color.gray;

    private Renderer robotRenderer;

    void Start()
    {
        robotRenderer = GetComponent<Renderer>();
        SetupRobotMaterials();
    }

    void SetupRobotMaterials()
    {
        if (robotRenderer != null)
        {
            // Determine material type based on robot part name
            string partName = transform.name.ToLower();

            if (partName.Contains("metal") || partName.Contains("joint") || partName.Contains("frame"))
            {
                SetupMetalMaterial(robotRenderer);
            }
            else if (partName.Contains("plastic") || partName.Contains("cover") || partName.Contains("panel"))
            {
                SetupPlasticMaterial(robotRenderer);
            }
            else if (partName.Contains("rubber") || partName.Contains("tire") || partName.Contains("pad"))
            {
                SetupRubberMaterial(robotRenderer);
            }
            else if (partName.Contains("glass") || partName.Contains("camera") || partName.Contains("lens"))
            {
                SetupGlassMaterial(robotRenderer);
            }
            else
            {
                // Default to metal for robot parts
                SetupMetalMaterial(robotRenderer);
            }
        }
    }

    void SetupMetalMaterial(Renderer renderer)
    {
        Material material = new Material(Shader.Find("HDRP/Lit"));

        // Metallic properties for metal parts
        material.SetFloat("_Metallic", baseMetallic);
        material.SetFloat("_Smoothness", 1.0f - baseRoughness); // Smoothness is inverse of roughness
        material.SetColor("_BaseColor", baseColor);

        // Add textures if available
        if (normalMap != null)
            material.SetTexture("_NormalMap", normalMap);

        if (metallicMap != null)
            material.SetTexture("_MetallicGlossMap", metallicMap);

        if (roughnessMap != null)
            material.SetTexture("_SmoothnessMap", roughnessMap);

        renderer.material = material;
    }

    void SetupPlasticMaterial(Renderer renderer)
    {
        Material material = new Material(Shader.Find("HDRP/Lit"));

        // Plastic properties (non-metallic, varying roughness)
        material.SetFloat("_Metallic", 0.0f);
        material.SetFloat("_Smoothness", 0.8f - baseRoughness * 0.5f);
        material.SetColor("_BaseColor", baseColor);

        // Plastic often has subtle subsurface scattering
        material.SetFloat("_SubsurfaceMask", 0.2f);

        renderer.material = material;
    }

    void SetupRubberMaterial(Renderer renderer)
    {
        Material material = new Material(Shader.Find("HDRP/Lit"));

        // Rubber properties (non-metallic, high roughness)
        material.SetFloat("_Metallic", 0.0f);
        material.SetFloat("_Smoothness", 0.1f); // Very rough
        material.SetColor("_BaseColor", Color.black); // Usually black rubber

        // Add slight subsurface scattering for soft appearance
        material.SetFloat("_SubsurfaceMask", 0.8f);

        renderer.material = material;
    }

    void SetupGlassMaterial(Renderer renderer)
    {
        Material material = new Material(Shader.Find("HDRP/Lit"));

        // Glass properties (transparent, high smoothness)
        material.SetFloat("_Metallic", 0.0f);
        material.SetFloat("_Smoothness", 0.98f); // Very smooth
        material.SetColor("_BaseColor", new Color(0.9f, 0.9f, 0.95f, 0.1f)); // Slightly tinted transparent
        material.SetFloat("_SurfaceType", 1); // Transparent
        material.SetFloat("_BlendMode", 1); // Alpha blend

        renderer.material = material;
    }

    // Dynamic material property adjustment
    public void AdjustMaterialProperties(float metallic, float roughness, Color color)
    {
        if (robotRenderer != null && robotRenderer.material != null)
        {
            Material mat = robotRenderer.material;
            mat.SetFloat("_Metallic", metallic);
            mat.SetFloat("_Smoothness", 1.0f - roughness);
            mat.SetColor("_BaseColor", color);
        }
    }

    // Material highlighting effect
    public IEnumerator HighlightMaterial(Color highlightColor, float duration = 0.5f)
    {
        if (robotRenderer != null)
        {
            Material originalMaterial = robotRenderer.material;
            Material highlightMaterial = new Material(originalMaterial);

            // Store original color
            Color originalColor = originalMaterial.GetColor("_BaseColor");

            // Apply highlight
            highlightMaterial.SetColor("_BaseColor", highlightColor);
            robotRenderer.material = highlightMaterial;

            yield return new WaitForSeconds(duration);

            // Restore original material
            robotRenderer.material = originalMaterial;
        }
    }
}
```

### Example 2: Advanced Lighting System Setup
Creating realistic lighting with shadows and global illumination:

```csharp
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

public class AdvancedLightingSystem : MonoBehaviour
{
    [Header("Light Configuration")]
    public Light mainDirectionalLight;
    public Light[] fillLights;
    public Light[] rimLights;

    [Header("Shadow Settings")]
    public ShadowResolution shadowResolution = ShadowResolution._2048;
    public float shadowDistance = 100f;
    public float shadowBias = 0.05f;
    public float shadowNormalBias = 0.4f;

    [Header("Global Illumination")]
    public bool enableBakedGI = true;
    public bool enableRealtimeGI = true;
    public float environmentIntensity = 1.0f;

    [Header("Reflection Probes")]
    public ReflectionProbe[] reflectionProbes;
    public Cubemap customReflectionCubemap;

    [Header("Lighting Presets")]
    public float indoorBrightness = 1.5f;
    public float outdoorBrightness = 2.0f;
    public Color indoorColor = Color.white;
    public Color outdoorColor = new Color(0.95f, 0.95f, 1.0f, 1.0f);

    void Start()
    {
        SetupAdvancedLighting();
        SetupReflectionProbes();
    }

    void SetupAdvancedLighting()
    {
        // Configure main directional light (sun/sky light)
        if (mainDirectionalLight != null)
        {
            mainDirectionalLight.type = LightType.Directional;
            mainDirectionalLight.intensity = outdoorBrightness;
            mainDirectionalLight.color = outdoorColor;

            // Configure shadows
            mainDirectionalLight.shadows = LightShadows.Soft;
            mainDirectionalLight.shadowResolution = shadowResolution;
            mainDirectionalLight.shadowDistance = shadowDistance;
            mainDirectionalLight.shadowBias = shadowBias;
            mainDirectionalLight.shadowNormalBias = shadowNormalBias;

            // HDRP-specific shadow settings
            if (mainDirectionalLight.TryGetComponent(out HDAdditionalLightData hdLight))
            {
                hdLight.shadowCascadeOption = HDShadowCascadeOption.NoCascades;
                hdLight.shadowCascade2Split = 0.33f;
                hdLight.shadowCascade4Split = new Vector3(0.067f, 0.2f, 0.467f);
            }
        }

        // Setup fill lights for softer shadows
        SetupFillLights();

        // Setup rim lights for dramatic edge lighting
        SetupRimLights();
    }

    void SetupFillLights()
    {
        for (int i = 0; i < fillLights.Length; i++)
        {
            Light fillLight = fillLights[i];
            if (fillLight != null)
            {
                fillLight.type = LightType.Point;
                fillLight.intensity = 0.3f;
                fillLight.color = Color.gray;
                fillLight.range = 15f;

                // Disable shadows for fill lights to avoid shadow conflicts
                fillLight.shadows = LightShadows.None;

                // Position fill lights strategically
                float angle = (i * 2 * Mathf.PI) / fillLights.Length;
                float radius = 8f;
                fillLight.transform.position = new Vector3(
                    Mathf.Cos(angle) * radius,
                    4f,
                    Mathf.Sin(angle) * radius
                );
            }
        }
    }

    void SetupRimLights()
    {
        for (int i = 0; i < rimLights.Length; i++)
        {
            Light rimLight = rimLights[i];
            if (rimLight != null)
            {
                rimLight.type = LightType.Spot;
                rimLight.intensity = 0.8f;
                rimLight.color = new Color(0.8f, 0.8f, 1.0f, 1.0f); // Cool blue-white
                rimLight.spotAngle = 45f;
                rimLight.range = 12f;

                // Configure rim lighting direction
                rimLight.transform.LookAt(Vector3.zero);
                rimLight.transform.position = new Vector3(
                    Mathf.Cos(i * Mathf.PI / rimLights.Length) * 10f,
                    3f,
                    Mathf.Sin(i * Mathf.PI / rimLights.Length) * 10f
                );

                rimLight.shadows = LightShadows.Hard; // Sharp rim shadows
            }
        }
    }

    void SetupReflectionProbes()
    {
        foreach (ReflectionProbe probe in reflectionProbes)
        {
            if (probe != null)
            {
                // Configure probe for robot environment
                probe.mode = ReflectionProbeMode.Realtime;
                probe.refreshMode = ReflectionProbeRefreshMode.OnAwake;
                probe.timeSlicingMode = ReflectionProbeTimeSlicingMode.AllFacesAtOnce;

                // Set probe bounds to capture robot and surroundings
                probe.size = new Vector3(20f, 10f, 20f);
                probe.center = new Vector3(0f, 2f, 0f);

                if (customReflectionCubemap != null)
                {
                    probe.customBakedTexture = customReflectionCubemap;
                }
            }
        }
    }

    // Dynamic lighting adjustments
    public void SwitchToIndoorLighting()
    {
        if (mainDirectionalLight != null)
        {
            mainDirectionalLight.intensity = indoorBrightness;
            mainDirectionalLight.color = indoorColor;
        }

        foreach (Light fillLight in fillLights)
        {
            if (fillLight != null)
            {
                fillLight.intensity = 0.5f; // Brighter indoors
                fillLight.color = new Color(0.95f, 0.9f, 0.8f, 1.0f); // Warm indoor light
            }
        }
    }

    public void SwitchToOutdoorLighting()
    {
        if (mainDirectionalLight != null)
        {
            mainDirectionalLight.intensity = outdoorBrightness;
            mainDirectionalLight.color = outdoorColor;
        }

        foreach (Light fillLight in fillLights)
        {
            if (fillLight != null)
            {
                fillLight.intensity = 0.3f; // Softer outdoors
                fillLight.color = Color.gray; // Neutral fill
            }
        }
    }

    // Dynamic shadow adjustment
    public void AdjustShadowSettings(float distance, float bias, float normalBias)
    {
        if (mainDirectionalLight != null)
        {
            mainDirectionalLight.shadowDistance = distance;
            mainDirectionalLight.shadowBias = bias;
            mainDirectionalLight.shadowNormalBias = normalBias;
        }
    }

    // Environment lighting intensity
    public void SetEnvironmentIntensity(float intensity)
    {
        RenderSettings.ambientIntensity = intensity;
        environmentIntensity = intensity;

        if (mainDirectionalLight != null)
        {
            mainDirectionalLight.intensity = outdoorBrightness * intensity;
        }
    }
}
```

### Example 3: Post-Processing Effects Setup
Implementing post-processing effects for enhanced visual quality:

```csharp
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

public class PostProcessingManager : MonoBehaviour
{
    [Header("Post-Processing Profiles")]
    public Volume mainVolume;
    public VolumeProfile cinematicProfile;
    public VolumeProfile realisticProfile;
    public VolumeProfile artisticProfile;

    [Header("Effect Intensities")]
    [Range(0f, 1f)] public float bloomIntensity = 0.5f;
    [Range(0f, 1f)] public float chromaticAberrationIntensity = 0.2f;
    [Range(0f, 1f)] public float vignetteIntensity = 0.3f;
    [Range(0f, 1f)] public float grainIntensity = 0.1f;

    [Header("Color Grading")]
    public ColorFilter colorFilter;
    public LiftGammaGain liftGammaGain;
    public SplitToning splitToning;

    [Header("Depth of Field")]
    public bool enableDOF = false;
    public float dofFocusDistance = 5f;
    public float dofAperture = 5.6f;
    public float dofFocalLength = 50f;

    private Volume currentVolume;
    private Bloom bloomEffect;
    private ChromaticAberration chromaticAberrationEffect;
    private Vignette vignetteEffect;
    private FilmGrain filmGrainEffect;
    private DepthOfField depthOfFieldEffect;

    void Start()
    {
        SetupPostProcessingEffects();
        ApplyCinematicProfile();
    }

    void SetupPostProcessingEffects()
    {
        // Create volume if not assigned
        if (mainVolume == null)
        {
            GameObject volumeGO = new GameObject("PostProcessingVolume");
            mainVolume = volumeGO.AddComponent<Volume>();
            volumeGO.transform.SetParent(transform);
        }

        // Get or create profile
        if (cinematicProfile == null)
        {
            cinematicProfile = ScriptableObject.CreateInstance<VolumeProfile>();
        }

        if (realisticProfile == null)
        {
            realisticProfile = ScriptableObject.CreateInstance<VolumeProfile>();
        }

        if (artisticProfile == null)
        {
            artisticProfile = ScriptableObject.CreateInstance<VolumeProfile>();
        }

        // Add effects to cinematic profile
        AddEffectsToProfile(cinematicProfile);
        AddEffectsToProfile(realisticProfile);
        AddEffectsToProfile(artisticProfile);

        currentVolume = mainVolume;
        currentVolume.sharedProfile = cinematicProfile;
    }

    void AddEffectsToProfile(VolumeProfile profile)
    {
        // Bloom effect - adds glow to bright areas
        if (profile.components.FirstOrDefault(c => c is Bloom) == null)
        {
            profile.Add<Bloom>(true);
        }

        // Chromatic Aberration - adds color fringing
        if (profile.components.FirstOrDefault(c => c is ChromaticAberration) == null)
        {
            profile.Add<ChromaticAberration>(true);
        }

        // Vignette - darkens corners
        if (profile.components.FirstOrDefault(c => c is Vignette) == null)
        {
            profile.Add<Vignette>(true);
        }

        // Film Grain - adds subtle noise
        if (profile.components.FirstOrDefault(c => c is FilmGrain) == null)
        {
            profile.Add<FilmGrain>(true);
        }

        // Depth of Field - focuses on specific distance
        if (profile.components.FirstOrDefault(c => c is DepthOfField) == null)
        {
            profile.Add<DepthOfField>(true);
        }

        // Color Grading - adjusts colors
        if (profile.components.FirstOrDefault(c => c is ColorAdjustments) == null)
        {
            profile.Add<ColorAdjustments>(true);
        }

        if (profile.components.FirstOrDefault(c => c is LiftGammaGain) == null)
        {
            profile.Add<LiftGammaGain>(true);
        }
    }

    void Update()
    {
        UpdateEffectParameters();
    }

    void UpdateEffectParameters()
    {
        // Update Bloom
        var bloom = currentVolume.profile.components.FirstOrDefault(c => c is Bloom) as Bloom;
        if (bloom != null)
        {
            bloom.threshold.value = 0.9f;
            bloom.intensity.value = bloomIntensity;
            bloom.scatter.value = 0.7f;
            bloom.clamp.value = 65472f;
        }

        // Update Chromatic Aberration
        var chromAberration = currentVolume.profile.components.FirstOrDefault(c => c is ChromaticAberration) as ChromaticAberration;
        if (chromAberration != null)
        {
            chromAberration.intensity.value = chromaticAberrationIntensity;
        }

        // Update Vignette
        var vignette = currentVolume.profile.components.FirstOrDefault(c => c is Vignette) as Vignette;
        if (vignette != null)
        {
            vignette.intensity.value = vignetteIntensity;
            vignette.smoothness.value = 0.2f;
            vignette.roundness.value = 1f;
        }

        // Update Film Grain
        var grain = currentVolume.profile.components.FirstOrDefault(c => c is FilmGrain) as FilmGrain;
        if (grain != null)
        {
            grain.intensity.value = grainIntensity;
            grain.response.value = 0.8f;
            grain.luminanceContribution.value = 0.8f;
        }

        // Update Depth of Field
        var dof = currentVolume.profile.components.FirstOrDefault(c => c is DepthOfField) as DepthOfField;
        if (dof != null && enableDOF)
        {
            dof.focusDistance.value = dofFocusDistance;
            dof.aperture.value = dofAperture;
            dof.focalLength.value = dofFocalLength;
            dof.kernelSize.value = KernelSize.Medium;
        }

        // Update Color Adjustments
        var colorAdjust = currentVolume.profile.components.FirstOrDefault(c => c is ColorAdjustments) as ColorAdjustments;
        if (colorAdjust != null)
        {
            colorAdjust.postExposure.value = 0.2f;
            colorAdjust.contrast.value = 10f;
            colorAdjust.colorFilter.value = Color.white;
        }
    }

    // Profile switching
    public void ApplyCinematicProfile()
    {
        if (currentVolume != null && cinematicProfile != null)
        {
            currentVolume.sharedProfile = cinematicProfile;
        }
    }

    public void ApplyRealisticProfile()
    {
        if (currentVolume != null && realisticProfile != null)
        {
            currentVolume.sharedProfile = realisticProfile;
        }
    }

    public void ApplyArtisticProfile()
    {
        if (currentVolume != null && artisticProfile != null)
        {
            currentVolume.sharedProfile = artisticProfile;
        }
    }

    // Dynamic effect control
    public void SetBloomIntensity(float intensity)
    {
        bloomIntensity = Mathf.Clamp(intensity, 0f, 1f);
    }

    public void SetChromaticAberrationIntensity(float intensity)
    {
        chromaticAberrationIntensity = Mathf.Clamp(intensity, 0f, 1f);
    }

    public void SetVignetteIntensity(float intensity)
    {
        vignetteIntensity = Mathf.Clamp(intensity, 0f, 1f);
    }

    public void SetGrainIntensity(float intensity)
    {
        grainIntensity = Mathf.Clamp(intensity, 0f, 1f);
    }

    public void ToggleDepthOfField(bool enable)
    {
        enableDOF = enable;
    }

    public void SetDOFFocusDistance(float distance)
    {
        dofFocusDistance = Mathf.Max(distance, 0.1f);
    }

    public void SetDOFAperture(float aperture)
    {
        dofAperture = Mathf.Clamp(aperture, 0.1f, 32f);
    }

    // Special effects for robot highlighting
    public void ApplyRobotHighlightingEffect()
    {
        // Increase bloom and adjust colors for a "highlighted" look
        StartCoroutine(RobotHighlightingCoroutine());
    }

    System.Collections.IEnumerator RobotHighlightingCoroutine()
    {
        float originalBloom = bloomIntensity;
        float originalVignette = vignetteIntensity;

        // Increase bloom and vignette temporarily
        bloomIntensity = Mathf.Min(bloomIntensity + 0.3f, 1.0f);
        vignetteIntensity = Mathf.Min(vignetteIntensity + 0.2f, 1.0f);

        yield return new WaitForSeconds(0.5f);

        // Gradually return to original values
        float duration = 1.0f;
        float elapsed = 0f;

        while (elapsed < duration)
        {
            bloomIntensity = Mathf.Lerp(bloomIntensity, originalBloom, elapsed / duration);
            vignetteIntensity = Mathf.Lerp(vignetteIntensity, originalVignette, elapsed / duration);
            elapsed += Time.deltaTime;
            yield return null;
        }

        bloomIntensity = originalBloom;
        vignetteIntensity = originalVignette;
    }

    // Performance optimization
    public void OptimizeForPerformance()
    {
        // Reduce expensive effects when performance is needed
        bloomIntensity = Mathf.Min(bloomIntensity, 0.3f);
        grainIntensity = 0f; // Disable grain for performance
        chromaticAberrationIntensity = Mathf.Min(chromaticAberrationIntensity, 0.1f);
    }

    public void RestoreQualitySettings()
    {
        // Restore original quality settings
        bloomIntensity = 0.5f;
        grainIntensity = 0.1f;
        chromaticAberrationIntensity = 0.2f;
    }
}
```

### Example 4: Rendering Optimization Manager
Optimizing rendering performance for real-time applications:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class RenderingOptimizationManager : MonoBehaviour
{
    [Header("LOD Settings")]
    public int lodCount = 3;
    public float[] lodDistances = { 10f, 25f, 50f };
    public bool enableLOD = true;

    [Header("Occlusion Culling")]
    public bool enableOcclusionCulling = true;
    public float occlusionCullingInterval = 0.1f;

    [Header("Rendering Quality")]
    public int targetFrameRate = 60;
    public float qualityThreshold = 0.8f; // Threshold for quality adjustments

    [Header("Dynamic Batching")]
    public bool enableDynamicBatching = true;
    public int maxBatchedVertices = 32000;

    [Header("Quality Presets")]
    public RenderingQuality lowQuality;
    public RenderingQuality mediumQuality;
    public RenderingQuality highQuality;

    private List<Renderer> robotRenderers = new List<Renderer>();
    private List<LODGroup> lodGroups = new List<LODGroup>();
    private float lastFrameTime;
    private int frameCount = 0;
    private float fpsTimer = 0f;

    [System.Serializable]
    public class RenderingQuality
    {
        public string name;
        public float shadowDistanceMultiplier = 1.0f;
        public float reflectionQuality = 1.0f;
        public float postProcessingIntensity = 1.0f;
        public float textureQuality = 1.0f;
    }

    void Start()
    {
        Application.targetFrameRate = targetFrameRate;
        lastFrameTime = Time.time;

        FindRobotComponents();
        SetupLODGroups();
        StartCoroutine(OcclusionCullingCoroutine());
    }

    void FindRobotComponents()
    {
        // Find all robot renderers in children
        Renderer[] renderers = GetComponentsInChildren<Renderer>();
        foreach (Renderer renderer in renderers)
        {
            if (renderer.CompareTag("RobotPart") ||
                renderer.name.ToLower().Contains("robot") ||
                renderer.name.ToLower().Contains("joint"))
            {
                robotRenderers.Add(renderer);
            }
        }

        // Find LOD groups
        LODGroup[] lodGroupsArray = GetComponentsInChildren<LODGroup>();
        foreach (LODGroup lodGroup in lodGroupsArray)
        {
            lodGroups.Add(lodGroup);
        }
    }

    void SetupLODGroups()
    {
        if (!enableLOD) return;

        foreach (Renderer renderer in robotRenderers)
        {
            // Create LOD group for complex robot parts
            if (renderer.GetComponent<LODGroup>() == null)
            {
                GameObject lodParent = new GameObject($"{renderer.name}_LOD");
                lodParent.transform.SetParent(renderer.transform.parent);
                lodParent.transform.position = renderer.transform.position;
                lodParent.transform.rotation = renderer.transform.rotation;

                LODGroup lodGroup = lodParent.AddComponent<LODGroup>();

                // Create LOD levels (simplified versions would be created in a real implementation)
                LOD[] lods = new LOD[lodCount];
                for (int i = 0; i < lodCount; i++)
                {
                    // In a real implementation, you would create simplified meshes for each LOD
                    float screenRelativeTransitionHeight = 1.0f - (i * 0.3f);
                    lods[i] = new LOD(screenRelativeTransitionHeight, new Renderer[] { renderer });
                }

                lodGroup.SetLODs(lods);
                lodGroup.RecalculateBounds();

                lodGroups.Add(lodGroup);
            }
        }
    }

    void Update()
    {
        MonitorPerformance();
        AdjustRenderingBasedOnPerformance();
    }

    void MonitorPerformance()
    {
        frameCount++;
        fpsTimer += Time.unscaledDeltaTime;

        if (fpsTimer >= 1.0f) // Every second
        {
            float currentFPS = frameCount / fpsTimer;
            float performanceRatio = currentFPS / targetFrameRate;

            // Adjust quality based on performance
            if (performanceRatio < qualityThreshold * 0.8f)
            {
                // Significantly below threshold - drop to low quality
                ApplyRenderingQuality(lowQuality);
            }
            else if (performanceRatio < qualityThreshold)
            {
                // Below threshold - use medium quality
                ApplyRenderingQuality(mediumQuality);
            }
            else
            {
                // Above threshold - use high quality
                ApplyRenderingQuality(highQuality);
            }

            frameCount = 0;
            fpsTimer = 0f;
        }
    }

    void AdjustRenderingBasedOnPerformance()
    {
        // Dynamic batching optimization
        if (enableDynamicBatching)
        {
            // Unity handles dynamic batching automatically, but we can influence it
            Shader.globalMaximumLOD = 600; // Higher LOD for detailed models
        }

        // Adjust shadow distances based on performance
        float currentFPS = 1.0f / Time.unscaledDeltaTime;
        float shadowDistanceScale = Mathf.Clamp(currentFPS / targetFrameRate, 0.5f, 1.0f);

        // Apply shadow distance scaling (would connect to lighting system)
        Light[] lights = FindObjectsOfType<Light>();
        foreach (Light light in lights)
        {
            if (light.shadows != LightShadows.None)
            {
                light.shadowDistance *= shadowDistanceScale;
            }
        }
    }

    void ApplyRenderingQuality(RenderingQuality quality)
    {
        // Apply shadow distance multiplier
        Light[] lights = FindObjectsOfType<Light>();
        foreach (Light light in lights)
        {
            if (light.shadows != LightShadows.None)
            {
                light.shadowDistance = 100f * quality.shadowDistanceMultiplier;
            }
        }

        // Apply texture quality scaling
        QualitySettings.masterTextureLimit = Mathf.RoundToInt((1.0f - quality.textureQuality) * 4);

        // Apply other quality settings as needed
        RenderSettings.reflectionBounces = Mathf.RoundToInt(quality.reflectionQuality * 3);
    }

    System.Collections.IEnumerator OcclusionCullingCoroutine()
    {
        while (enableOcclusionCulling)
        {
            // Perform occlusion culling calculations
            PerformOcclusionCulling();

            yield return new WaitForSeconds(occlusionCullingInterval);
        }
    }

    void PerformOcclusionCulling()
    {
        // Unity's built-in occlusion culling works automatically when enabled in the scene
        // This method could be extended with custom occlusion culling logic if needed
        Camera mainCam = Camera.main;
        if (mainCam != null)
        {
            // Calculate which robot parts are visible to the camera
            foreach (Renderer renderer in robotRenderers)
            {
                if (renderer != null)
                {
                    // Check if renderer is in camera's view frustum
                    bool isVisible = GeometryUtility.TestPlanesAABB(
                        GeometryUtility.CalculateFrustumPlanes(mainCam),
                        renderer.bounds
                    );

                    // Enable/disable renderer based on visibility
                    renderer.enabled = isVisible;
                }
            }
        }
    }

    // Distance-based optimizations
    public void OptimizeBasedOnDistance(Transform viewer, float maxDistance = 100f)
    {
        foreach (Renderer renderer in robotRenderers)
        {
            if (renderer != null)
            {
                float distance = Vector3.Distance(viewer.position, renderer.transform.position);

                if (distance > maxDistance)
                {
                    // Disable rendering for distant objects
                    renderer.enabled = false;
                }
                else if (distance > maxDistance * 0.7f)
                {
                    // Reduce quality for far objects
                    renderer.material.SetFloat("_Smoothness", 0.3f); // Less shiny
                }
                else
                {
                    // Full quality for close objects
                    renderer.enabled = true;
                }
            }
        }
    }

    // Quality switching methods
    public void SetLowQuality()
    {
        ApplyRenderingQuality(lowQuality);
        QualitySettings.SetQualityLevel(2); // Low quality preset
    }

    public void SetMediumQuality()
    {
        ApplyRenderingQuality(mediumQuality);
        QualitySettings.SetQualityLevel(3); // Medium quality preset
    }

    public void SetHighQuality()
    {
        ApplyRenderingQuality(highQuality);
        QualitySettings.SetQualityLevel(5); // High quality preset
    }

    // Memory optimization
    public void UnloadUnusedAssets()
    {
        // Unload unused assets to free up memory
        Resources.UnloadUnusedAssets();
        System.GC.Collect();
    }
}
```

## Simulation Exercises

### Exercise 1: Material Property Optimization
- **Objective**: Create realistic materials for different robot components
- **Requirements**: Unity with HDRP, 3D modeling knowledge
- **Steps**:
  1. Create materials for metal, plastic, and rubber robot parts
  2. Configure PBR properties (metallic, roughness, normal maps)
  3. Test materials under different lighting conditions
  4. Optimize material properties for visual appeal
- **Expected Outcome**: Realistic robot materials with proper PBR properties

### Exercise 2: Advanced Lighting Setup
- **Objective**: Implement realistic lighting with shadows and reflections
- **Requirements**: Unity with lighting knowledge, HDRP
- **Steps**:
  1. Set up main directional light with proper shadows
  2. Add fill lights to soften shadows
  3. Configure reflection probes for robot reflections
  4. Test lighting under indoor and outdoor conditions
- **Expected Outcome**: Professional-quality lighting setup with realistic shadows and reflections

### Exercise 3: Post-Processing Enhancement
- **Objective**: Apply post-processing effects to enhance visual quality
- **Requirements**: Unity with Post-Processing Stack, Volume system
- **Steps**:
  1. Set up post-processing volumes with bloom and color grading
  2. Configure depth of field for selective focus
  3. Add film grain and chromatic aberration for cinematic feel
  4. Test performance impact and optimize settings
- **Expected Outcome**: Enhanced visual quality with appropriate post-processing effects

## Summary

High-fidelity rendering and lighting in Unity involve implementing advanced techniques including Physically-Based Rendering (PBR), realistic lighting systems with shadows and reflections, post-processing effects, and performance optimization strategies. The combination of proper material properties, realistic lighting setups, and cinematic post-processing effects creates photorealistic visualizations that accurately represent robot appearance and environmental interactions. Balancing visual quality with real-time performance requires careful consideration of rendering techniques, Level of Detail (LOD) systems, and dynamic quality adjustments based on performance metrics.

## Key Terms

- **PBR (Physically-Based Rendering)**: Rendering approach based on physical properties of materials
- **Global Illumination**: System that simulates light bouncing around the environment
- **Shadow Mapping**: Technique for rendering realistic shadows
- **Reflection Probes**: Components that capture environment reflections
- **Post-Processing**: Effects applied after rendering to enhance visual quality
- **Bloom Effect**: Glowing effect for bright areas in the scene
- **Depth of Field**: Focus effect that blurs out-of-focus areas
- **LOD (Level of Detail)**: System that uses simpler models at greater distances

## Next Steps

Continue to Lesson 6.3: "Interactive Visualization & User Experience" to explore how to create engaging user interfaces and interactive experiences for robot visualization.