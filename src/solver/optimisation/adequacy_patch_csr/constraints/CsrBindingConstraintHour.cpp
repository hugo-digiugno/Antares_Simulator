// Copyright 2007-2026, RTE (https://www.rte-france.com)
// SPDX-License-Identifier: MPL-2.0

#include <antares/logs/logs.h>

#include "antares/solver/optimisation/adequacy_patch_csr/constraints/CsrBindingConstraintHour.h"

void CsrBindingConstraintHour::add(int CntCouplante)
{
    if (data.MatriceDesContraintesCouplantes[CntCouplante].TypeDeContrainteCouplante
        != CONTRAINTE_HORAIRE)
    {
        return;
    }

    int NbInterco = data.MatriceDesContraintesCouplantes[CntCouplante]
                      .NombreDInterconnexionsDansLaContrainteCouplante;
    builder.updateHourWithinWeek(data.hour);

    for (int Index = 0; Index < NbInterco; Index++)
    {
        int Interco = data.MatriceDesContraintesCouplantes[CntCouplante]
                        .NumeroDeLInterconnexion[Index];
        double Poids = data.MatriceDesContraintesCouplantes[CntCouplante]
                         .PoidsDeLInterconnexion[Index];

        const bool originInside
          = data.originAreaMode[Interco] == Data::AdequacyPatch::physicalAreaInsideAdqPatch;
        const bool extremityInside
          = data.extremityAreaMode[Interco] == Data::AdequacyPatch::physicalAreaInsideAdqPatch;

        if (originInside && extremityInside)
        {
            builder.NTCDirect(Interco, Poids);
        }
        else
        {
            Antares::logs.info()
              << "[ADQ-DEBUG][ORG-BC-DROP] h=" << data.hour
              << " cnec=" << CntCouplante
              << " interco=" << Interco
              << " ptdf=" << Poids
              << " originMode=" << static_cast<int>(data.originAreaMode[Interco])
              << " extremityMode=" << static_cast<int>(data.extremityAreaMode[Interco])
              << " (outside-area term excluded from CSR binding constraint)";
        }
    }

    if (builder.NumberOfVariables()
        > 0) // current binding constraint contains an interco type 2<->2
    {
        data.numberOfConstraintCsrHourlyBinding[CntCouplante] = builder.data.nombreDeContraintes;

        ConstraintNamer namer(builder.data.NomDesContraintes);
        namer.UpdateTimeStep(data.hour);
        namer.BindingConstraintHour(
          builder.data.nombreDeContraintes,
          data.MatriceDesContraintesCouplantes[CntCouplante].NomDeLaContrainteCouplante);
        builder.SetOperator(
          data.MatriceDesContraintesCouplantes[CntCouplante].SensDeLaContrainteCouplante);
        builder.build();
    }
}
