// Copyright LoogLong. All Rights Reserved.
#include "LoogPhysicsEditor.h"
#include "Modules/ModuleManager.h"
#include "Textures/SlateIcon.h"
#include "LoogPhysicsEditMode.h"

#define LOCTEXT_NAMESPACE "LoogPhysicsEditorModule"


void FLoogPhysicsEditorModule::StartupModule()
{
	FEditorModeRegistry::Get().RegisterMode<FLoogPhysicsEditMode>(FLoogPhysicsEditMode::ModeName, LOCTEXT("FLoogPhysicsEditMode", "Loog Physics"), FSlateIcon(), false);
}


void FLoogPhysicsEditorModule::ShutdownModule()
{
	FEditorModeRegistry::Get().UnregisterMode(FLoogPhysicsEditMode::ModeName);
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FLoogPhysicsEditorModule, LoogPhysicsEditor)
