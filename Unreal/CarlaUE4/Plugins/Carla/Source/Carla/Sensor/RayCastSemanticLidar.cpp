// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Sensor/RayCastSemanticLidar.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/geom/Vector4DuInt.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "Runtime/Engine/Classes/Kismet/KismetRenderingLibrary.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "Runtime/Engine/Classes/Kismet/GameplayStatics.h"

namespace crp = carla::rpc;

FActorDefinition ARayCastSemanticLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast_semantic"));
}

ARayCastSemanticLidar::ARayCastSemanticLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
}

void ARayCastSemanticLidar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void ARayCastSemanticLidar::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  SemanticLidarData = FSemanticLidarData(Description.Channels);
  CreateLasers();
  PointsPerChannel.resize(Description.Channels);
}

void ARayCastSemanticLidar::CreateLasers()
{
  const auto NumberOfLasers = Description.Channels;
  check(NumberOfLasers > 0u);
  const float DeltaAngle = NumberOfLasers == 1u ? 0.f :
    (Description.UpperFovLimit - Description.LowerFovLimit) /
    static_cast<float>(NumberOfLasers - 1);
  LaserAngles.Empty(NumberOfLasers);
  for(auto i = 0u; i < NumberOfLasers; ++i)
  {
    const float VerticalAngle =
        Description.UpperFovLimit - static_cast<float>(i) * DeltaAngle;
    LaserAngles.Emplace(VerticalAngle);
  }
}

void ARayCastSemanticLidar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastSemanticLidar::PostPhysTick);
  SimulateLidar(DeltaTime);

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    auto DataStream = GetDataStream(*this);
    DataStream.Send(*this, SemanticLidarData, DataStream.PopBufferFromPool());
  }
}

void ARayCastSemanticLidar::SimulateLidar(const float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastSemanticLidar::SimulateLidar);
  const uint32 ChannelCount = Description.Channels;
  const uint32 PointsToScanWithOneLaser =
    FMath::RoundHalfFromZero(
        Description.PointsPerSecond * DeltaTime / float(ChannelCount));

  if (PointsToScanWithOneLaser <= 0)
  {
    UE_LOG(
        LogCarla,
        Warning,
        TEXT("%s: no points requested this frame, try increasing the number of points per second."),
        *GetName());
    return;
  }

  check(ChannelCount == LaserAngles.Num());

  const float CurrentHorizontalAngle = carla::geom::Math::ToDegrees(
      SemanticLidarData.GetHorizontalAngle());
  const float AngleDistanceOfTick = Description.RotationFrequency * Description.HorizontalFov
      * DeltaTime;
  const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;

  ResetRecordedHits(ChannelCount, PointsToScanWithOneLaser);
  PreprocessRays(ChannelCount, PointsToScanWithOneLaser);

  GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
  {
    TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
    ParallelFor(ChannelCount, [&](int32 idxChannel) {
      TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);

      FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
      TraceParams.bTraceComplex = true;
      TraceParams.bReturnFaceIndex = true;
      TraceParams.bReturnPhysicalMaterial = true;

      for (auto idxPtsOneLaser = 0u; idxPtsOneLaser < PointsToScanWithOneLaser; idxPtsOneLaser++) {
        FHitResult HitResult;
        const float VertAngle = LaserAngles[idxChannel];
        const float HorizAngle = std::fmod(CurrentHorizontalAngle + AngleDistanceOfLaserMeasure
            * idxPtsOneLaser, Description.HorizontalFov) - Description.HorizontalFov / 2;
        // const bool PreprocessResult = RayPreprocessCondition[idxChannel][idxPtsOneLaser];

        // if (PreprocessResult && ShootLaser(VertAngle, HorizAngle, HitResult, TraceParams)) {
        //  WritePointAsync(idxChannel, HitResult);
        //}
        //
        // always write pt
    ShootLaser(VertAngle, HorizAngle, HitResult, TraceParams);
        WritePointAsync(idxChannel, HitResult);
      };
    });
  }
  GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

  FTransform ActorTransf = GetTransform();
  ComputeAndSaveDetections(ActorTransf);

  const float HorizontalAngle = carla::geom::Math::ToRadians(
      std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, Description.HorizontalFov));
  SemanticLidarData.SetHorizontalAngle(HorizontalAngle);
}

void ARayCastSemanticLidar::ResetRecordedHits(uint32_t Channels, uint32_t MaxPointsPerChannel) {
  RecordedHits.resize(Channels);

  for (auto& hits : RecordedHits) {
    hits.clear();
    hits.reserve(MaxPointsPerChannel);
  }
}

void ARayCastSemanticLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
  RayPreprocessCondition.resize(Channels);

  for (auto& conds : RayPreprocessCondition) {
    conds.clear();
    conds.resize(MaxPointsPerChannel);
    std::fill(conds.begin(), conds.end(), true);
  }
}

void ARayCastSemanticLidar::WritePointAsync(uint32_t channel, FHitResult &detection) {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
  DEBUG_ASSERT(GetChannelCount() > channel);
  RecordedHits[channel].emplace_back(detection);
}

void ARayCastSemanticLidar::ComputeAndSaveDetections(const FTransform& SensorTransform) {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
    PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();
  SemanticLidarData.ResetMemory(PointsPerChannel);

  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
    for (auto& hit : RecordedHits[idxChannel]) {
      FSemanticDetection detection;
      ComputeRawDetection(hit, SensorTransform, detection);
      SemanticLidarData.WritePointSync(detection);
    }
  }

  SemanticLidarData.WriteChannelCount(PointsPerChannel);
}

void ARayCastSemanticLidar::ComputeRawDetection(const FHitResult& HitInfo, const FTransform& SensorTransf, FSemanticDetection& Detection) const
{
    // always keep angles
    Detection.point = FVector(HitInfo.TraceEnd.X, HitInfo.TraceEnd.Y, HitInfo.TraceEnd.Z);
    UE_LOG(LogCarla, Warning, TEXT(" "));
    if (HitInfo.bBlockingHit) {
        // now that lidar returns hit angles and distances
        // do not modify the inpact point...
        // something was actually hit
        const FVector HitPoint = HitInfo.ImpactPoint;
        // Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);EMaterialProperty
        // use angles instead of detection point in cartesian coordinates

        const FVector VecInc = - (HitPoint - SensorTransf.GetLocation()).GetSafeNormal();
        Detection.cos_inc_angle = FVector::DotProduct(VecInc, HitInfo.ImpactNormal);

        // const FActorRegistry &Registry = GetEpisode().GetActorRegistry();
        // const AActor* actor = HitInfo.Actor.Get();
        Detection.object_idx = 0;
        Detection.object_tag = static_cast<uint32_t>(HitInfo.Component->CustomDepthStencilValue);
        UStaticMeshComponent * comp = Cast<UStaticMeshComponent>(HitInfo.GetComponent());  // try to cast...
        // int32 numUVChannels = mesh->GetNumUVChannels(0);  // LODIndex = 0 -> highest resolution
        if (comp != nullptr){
            ComputeRawDetectionFromComponent(HitInfo, Detection, comp);
        }
        else{
            // error code 1  component is nullptr
            UE_LOG(LogCarla, Warning, TEXT("Component is NULLPTR"));
            Detection.base_color = carla::geom::Vector4DuInt(1, 0, 0, 0);
            Detection.ORME = carla::geom::Vector4DuInt(0, 0, 0, 0);
            Detection.object_idx = 1;
        }
    }
    else {
        // nothing was hit within range
        // set everything to but keep angles
        Detection.cos_inc_angle = 2.0;  // actually impossible haha (might be easier to parse afterwards)
        Detection.object_idx = 0;
        Detection.object_tag = 0;
        Detection.base_color = carla::geom::Vector4DuInt(0, 0, 0, 0);
        Detection.ORME = carla::geom::Vector4DuInt(0, 0, 0, 0);
    }
}
void ARayCastSemanticLidar::ComputeRawDetectionFromComponent(
        const FHitResult &HitInfo, FSemanticDetection &Detection, UPrimitiveComponent * Component) const
{
    if (HitInfo.FaceIndex == -1){
        // error code 2
        UE_LOG(LogCarla, Warning, TEXT("Comp = %s"), *Component->GetName());
        UE_LOG(LogCarla, Warning, TEXT("Faceidx == -1"));
        Detection.base_color = carla::geom::Vector4DuInt(0, 0, 0, 0);
        Detection.ORME = carla::geom::Vector4DuInt(0, 0, 0, 0);
        Detection.object_idx = 2;
        return;
    }
    // get UV coords for the mesh we hit
    FVector2D uv_coords;
    // int32 uv_channel = 0;  // need to check if it is the right one??
    bool result = UGameplayStatics::FindCollisionUV(HitInfo, 0, uv_coords);
    if (!result){
        // error code 3 UV result is False
        UE_LOG(LogCarla, Warning, TEXT("Comp = %s"), *Component->GetName());
        UE_LOG(LogCarla, Warning, TEXT("UV result is False"));
        Detection.base_color = carla::geom::Vector4DuInt(0, 0, 0, 0);
        Detection.ORME = carla::geom::Vector4DuInt(0, 0, 0, 0);
        Detection.object_idx = 3;
        return;
    }
    // get material Interface
    int32 sectionIndex;
    UMaterialInterface * matInt = Component->GetMaterialFromCollisionFaceIndex(
            HitInfo.FaceIndex, sectionIndex);
    if (matInt == nullptr){
        // error code 4
        UE_LOG(LogCarla, Warning, TEXT("Comp = %s"), *Component->GetName());
        UE_LOG(LogCarla, Warning, TEXT("Material Interface is NULLPTR"));
        Detection.base_color = carla::geom::Vector4DuInt(0, 0, 0, 0);
        Detection.ORME = carla::geom::Vector4DuInt(0, 0, 0, 0);
        Detection.object_idx = 4;
        return;
    }
    // get material instance
    UMaterialInstance * materialInstance = Cast<UMaterialInstance>(matInt);
    if (materialInstance == nullptr){
        // try to get material instance in another way
        // error code 5 material instance == nullptr
        UE_LOG(LogCarla, Warning, TEXT("Material Instance is NULLPTR, comp = %s"), *Component->GetName());
        UE_LOG(LogCarla, Warning, TEXT("Material Instance is NULLPTR, material interface = %s"), *matInt->GetName());
        Detection.base_color = carla::geom::Vector4DuInt(0, 0, 0, 0);
        Detection.ORME = carla::geom::Vector4DuInt(0, 0, 0, 0);
        Detection.object_idx = 5;
        return;
    }
    // Now actually compute the detection parameters
    ComputeRawDetectionFromMaterialInstance(
            Detection, materialInstance, Component);
}
void ARayCastSemanticLidar::ComputeRawDetectionFromMaterialInstance(
        FSemanticDetection &Detection, UMaterialInstance * materialInstance,
        UPrimitiveComponent * Component) const
{
    // auto * parameterValues = &(materialInstance->TextureParameterValues);
    FColor textureColor = FColor(0, 0, 0, 0); 
    UTexture2D * texture;
    switch (materialInstance->TextureParameterValues.Num()){
        case 0:
            // no textures -> check scalar parameters
            switch (materialInstance->ScalarParameterValues.Num()){
                case 0:
                    // no scalar parameters either -> check vector parameters
                    UE_LOG(LogCarla, Warning, TEXT("Component is %s"), *Component->GetName());
                    UE_LOG(LogCarla, Warning, TEXT("material instance = %s"), *materialInstance->GetName());
                    switch (materialInstance->VectorParameterValues.Num()){
                        case 0:
                            UE_LOG(LogCarla, Warning, TEXT("No parameter values at all!!!"))
                            // error code 6
                            Detection.base_color = carla::geom::Vector4DuInt(0, 0, 0, 0);
                            Detection.ORME = carla::geom::Vector4DuInt(0, 0, 0, 0);
                            Detection.object_idx = 6;
                            break;
                        default:
                            // list them for now
                            for (int iparam = 0; iparam < materialInstance->ScalarParameterValues.Num(); iparam++){
                                UE_LOG(LogCarla, Warning, TEXT(" - %s"), *materialInstance->ScalarParameterValues[iparam].ParameterInfo.ToString());
                            }
                            UE_LOG(LogCarla, Fatal, TEXT(" vector parameters..."));
                            // error code 7
                            Detection.base_color = carla::geom::Vector4DuInt(0, 0, 0, 0);
                            Detection.ORME = carla::geom::Vector4DuInt(0, 0, 0, 0);
                            Detection.object_idx = 7;
                            break;
                    }
                    break;
                default:
                    // check if first scalar texture is transparency
                    if (materialInstance->ScalarParameterValues[0].ParameterInfo.Name == "Transparency"){
                        // store transparency as alpha and all other colors to white I guess...
                        Detection.ORME = carla::geom::Vector4DuInt(textureColor.R, textureColor.G, textureColor.B, textureColor.A);
                        uint8_t alpha = static_cast<uint8_t>(materialInstance->ScalarParameterValues[0].ParameterValue * 256);
                        // the alpha above have 0 -> non transparent -> should have an alpha of 255
                        Detection.base_color = carla::geom::Vector4DuInt(textureColor.R, textureColor.G, textureColor.B, 255 - alpha);
                        break;
                    }
                    else{
                        UE_LOG(LogCarla, Warning, TEXT("material instance = %s"), *materialInstance->GetName());
                        UE_LOG(LogCarla, Warning, TEXT("first scalar value is not transparency..."));
                        // error code 8
                        Detection.base_color = carla::geom::Vector4DuInt(0, 0, 0, 0);
                        Detection.ORME = carla::geom::Vector4DuInt(0, 0, 0, 0);
                        Detection.object_idx = 8;
                        return;
                    }
            }
            break;
        case 1:  // only base color
        case 2:
            // base color + normal
            Detection.ORME = carla::geom::Vector4DuInt(textureColor.R, textureColor.G, textureColor.B, textureColor.A);
            texture = Cast<UTexture2D>(materialInstance->TextureParameterValues[0].ParameterValue);
            // GetColorFromTexture(texture, uv_coords, textureColor);
            Detection.base_color = carla::geom::Vector4DuInt(textureColor.R, textureColor.G, textureColor.B, textureColor.A);
            break;
        case 3:
        case 4:
        case 13:  // roads are blends of the same 4 textures on top of global road mask
            // base color + normal + metallic/roughness
            // FTextureParameterValue * difuse = &(parameterValues[0]);
            texture = Cast<UTexture2D>(materialInstance->TextureParameterValues[0].ParameterValue);
            // GetColorFromTexture(texture, uv_coords, textureColor);
            Detection.base_color = carla::geom::Vector4DuInt(textureColor.R, textureColor.G, textureColor.B, textureColor.A);
            texture = Cast<UTexture2D>(materialInstance->TextureParameterValues[2].ParameterValue);
            // GetColorFromTexture(texture, uv_coords, textureColor);
            Detection.ORME = carla::geom::Vector4DuInt(textureColor.R, textureColor.G, textureColor.B, textureColor.A);
            break;
        default:
            UE_LOG(LogCarla, Warning, TEXT("Num texture parameter values not in {1, 2, 3, 4, 13} (%d)"), materialInstance->TextureParameterValues.Num());
            UE_LOG(LogCarla, Warning, TEXT("comp = %s"), *Component->GetName());
            UE_LOG(LogCarla, Warning, TEXT("Material Instance is %s"), *materialInstance->GetName());
            // list them
            for (int itex = 0; itex < materialInstance->TextureParameterValues.Num(); itex++){
                UE_LOG(LogCarla, Warning, TEXT("  - %s"), *materialInstance->TextureParameterValues[itex].ParameterValue->GetName());
            }
            UE_LOG(LogCarla, Fatal, TEXT(" "));
            break;
    }
}

void ARayCastSemanticLidar::GetColorFromTexture(UTexture2D * texture, const FVector2D uvCoordinates, FColor &color) const {
    // to avoid the lock to return nullptr
    TextureCompressionSettings OldCompressionSettings = texture->CompressionSettings;
#if WITH_EDITORONLY_DATA
    TextureMipGenSettings OldMipGenSettings = texture->MipGenSettings;
    texture->MipGenSettings = TextureMipGenSettings::TMGS_NoMipmaps;
#endif
    bool OldSRGB = texture->SRGB;
    texture->CompressionSettings = TextureCompressionSettings::TC_VectorDisplacementmap;
    texture->SRGB = false;
    texture->UpdateResource();
    // now get pixel value
    FTexture2DMipMap * mipmap = &texture->PlatformData->Mips[0];
    FByteBulkData * RawImageData = &mipmap->BulkData;
    FColor* FormatedImageData = reinterpret_cast<FColor*>(RawImageData->Lock(LOCK_READ_ONLY));
    if (FormatedImageData == nullptr){
        // https://isaratech.com/ue4-reading-the-pixels-from-a-utexture2d/
        UE_LOG(LogCarla, Fatal, TEXT("Lock returned nullptr..."));
    }
        uint32 width = mipmap->SizeX;
        uint32 height = mipmap->SizeY;
    uint8 PixelX = static_cast<uint8>(roundf(uvCoordinates[0] * width));
    uint8 PixelY = static_cast<uint8>(roundf(uvCoordinates[1] * height));
    if (PixelX >= width || PixelY >= height){
        UE_LOG(LogCarla, Fatal, TEXT("Overflow UV coordinates..."));
    }
    if (PixelX >= 0 && PixelY >= 0){
        color = FormatedImageData[PixelY * width + PixelX];
    }
    else{
        UE_LOG(LogCarla, Fatal, TEXT("Pixel coordinates invalid..."));
    }
    RawImageData->Unlock();
    // return previous settings
    texture->CompressionSettings = OldCompressionSettings;
#if WITH_EDITORONLY_DATA
    texture->MipGenSettings = OldMipGenSettings;
#endif
    texture->SRGB = OldSRGB;
    texture->UpdateResource();
}

bool ARayCastSemanticLidar::ShootLaser(const float VerticalAngle, const float HorizontalAngle, FHitResult& HitResult, FCollisionQueryParams& TraceParams) const
{
  TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

  FHitResult HitInfo(ForceInit);

  FTransform ActorTransf = GetTransform();
  FVector LidarBodyLoc = ActorTransf.GetLocation();
  FRotator LidarBodyRot = ActorTransf.Rotator();

  FRotator LaserRot (VerticalAngle, HorizontalAngle, 0);  // float InPitch, float InYaw, float InRoll
  FRotator ResultRot = UKismetMathLibrary::ComposeRotators(
    LaserRot,
    LidarBodyRot
  );

  const auto Range = Description.Range;
  FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;

  GetWorld()->ParallelLineTraceSingleByChannel(
    HitInfo,
    LidarBodyLoc,
    EndTrace,
    ECC_GameTraceChannel2,
    TraceParams,
    FCollisionResponseParams::DefaultResponseParam
  );
  HitResult = HitInfo;
  // use TraceEnd to store angles in the end
  if (HitResult.bBlockingHit) {
    // set hit point to angles instead of cartesian coordinates of impact
    // (spherical coordinates here)
    // HitResult.ImpactPoint.Set(VerticalAngle, HorizontalAngle, 0.0);
    // a hit was registered => store distance as well
    HitResult.TraceEnd.Set(VerticalAngle, HorizontalAngle, HitResult.Distance);
  }
  else {
    // No hit => set distance to 0
    HitResult.TraceEnd.Set(VerticalAngle, HorizontalAngle, 0.0);
    // HitResult.ImpactPoint.Set(VerticalAngle, HorizontalAngle, HitResult.Distance);
  }
  return true;
  // if (HitInfo.bBlockingHit) {
  //   HitResult = HitInfo;
  //   return true;
  // } else {
  //   return false;
  // }
}
