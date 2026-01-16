// Copyright LoogLong. All Rights Reserved.
#pragma once

#include "CoreMinimal.h"

class FLoogPhysicsEditorModule : public IModuleInterface
{
public:
	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
