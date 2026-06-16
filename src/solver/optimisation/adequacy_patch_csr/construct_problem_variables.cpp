// Copyright 2007-2026, RTE (https://www.rte-france.com)
// SPDX-License-Identifier: MPL-2.0

#include "antares/solver/optimisation/adequacy_patch_csr/hourly_csr_problem.h"
#include "antares/solver/optimisation/opt_structure_probleme_a_resoudre.h"
#include "antares/solver/simulation/adequacy_patch_runtime_data.h"
#include "antares/solver/simulation/sim_structure_probleme_economique.h"

#include "pi_constantes_externes.h"

void HourlyCSRProblem::constructVariableENS()
{
    int& NumberOfVariables = problemeAResoudre_.NombreDeVariables;
    NumberOfVariables = 0;

    // variables: ENS of each area inside adq patch
    logs.debug() << " ENS of each area inside adq patch: ";
    for (uint32_t area = 0; area < problemeHebdo_->NombreDePays; ++area)
    {
        // Only ENS for areas inside adq patch are considered as variables
        if (problemeHebdo_->adequacyPatchRuntimeData->areaMode[area]
            == Data::AdequacyPatch::physicalAreaInsideAdqPatch)
        {
            variableManager_.PositiveUnsuppliedEnergy(area, triggeredHour) = NumberOfVariables;
            problemeAResoudre_.TypeDeVariable[NumberOfVariables] = VARIABLE_BORNEE_DES_DEUX_COTES;
            problemeAResoudre_.NomDesVariables[NumberOfVariables]
              = std::string("ENS[") + problemeHebdo_->NomsDesPays[area] + "]";
            varToBeSetToZeroIfBelowThreshold.insert(NumberOfVariables);
            ensVariablesInsideAdqPatch.insert(NumberOfVariables);
            logs.debug() << NumberOfVariables << " ENS[" << area << "].-["
                         << problemeHebdo_->NomsDesPays[area] << "].";

            NumberOfVariables++;
        }
    }
}

void HourlyCSRProblem::constructVariableSpilledEnergy()
{
    int& NumberOfVariables = problemeAResoudre_.NombreDeVariables;

    // variables: Spilled Energy  of each area inside adq patch
    logs.debug() << " Spilled Energy  of each area inside adq patch: ";
    for (uint32_t area = 0; area < problemeHebdo_->NombreDePays; ++area)
    {
        // Only Spilled Energy  for areas inside adq patch are considered as variables
        if (problemeHebdo_->adequacyPatchRuntimeData->areaMode[area]
            == Data::AdequacyPatch::physicalAreaInsideAdqPatch)
        {
            variableManager_.NegativeUnsuppliedEnergy(area, triggeredHour) = NumberOfVariables;
            problemeAResoudre_.TypeDeVariable[NumberOfVariables] = VARIABLE_BORNEE_INFERIEUREMENT;
            problemeAResoudre_.NomDesVariables[NumberOfVariables]
              = std::string("SPILL[") + problemeHebdo_->NomsDesPays[area] + "]";
            varToBeSetToZeroIfBelowThreshold.insert(NumberOfVariables);
            logs.debug() << NumberOfVariables << " Spilled Energy[" << area << "].-["
                         << problemeHebdo_->NomsDesPays[area] << "].";

            NumberOfVariables++;
        }
    }
}

void HourlyCSRProblem::constructVariableFlows()
{
    int& NumberOfVariables = problemeAResoudre_.NombreDeVariables;

    // variables: transmissin flows (flow, direct_direct and flow_indirect). For links between 2
    // and 2.
    logs.debug()
      << " transmissin flows (flow, flow_direct and flow_indirect). For links between 2 and 2:";
    for (uint32_t Interco = 0; Interco < problemeHebdo_->NombreDInterconnexions; Interco++)
    {
        // only consider link between 2 and 2
        if (problemeHebdo_->adequacyPatchRuntimeData->originAreaMode[Interco]
              == Antares::Data::AdequacyPatch::physicalAreaInsideAdqPatch
            && problemeHebdo_->adequacyPatchRuntimeData->extremityAreaMode[Interco]
                 == Antares::Data::AdequacyPatch::physicalAreaInsideAdqPatch)
        {
            int algebraicFluxVar;
            int directVar;
            int indirectVar;
            algebraicFluxVar = variableManager_.NTCDirect(Interco, triggeredHour)
              = NumberOfVariables;
            {
                const std::string orig(
                  problemeHebdo_
                    ->NomsDesPays[problemeHebdo_->PaysOrigineDeLInterconnexion[Interco]]);
                const std::string extr(
                  problemeHebdo_
                    ->NomsDesPays[problemeHebdo_->PaysExtremiteDeLInterconnexion[Interco]]);
                problemeAResoudre_.NomDesVariables[NumberOfVariables]
                  = "FLOW[" + orig + ">>" + extr + "]";
                problemeAResoudre_.TypeDeVariable[NumberOfVariables]
                  = VARIABLE_BORNEE_DES_DEUX_COTES;
                logs.debug() << NumberOfVariables << " flow[" << Interco << "]. [" << extr
                             << "]-[" << orig << "].";
            }
            NumberOfVariables++;

            directVar = variableManager_.IntercoDirectCost(Interco, triggeredHour)
              = NumberOfVariables;
            problemeAResoudre_.TypeDeVariable[NumberOfVariables] = VARIABLE_BORNEE_DES_DEUX_COTES;
            problemeAResoudre_.NomDesVariables[NumberOfVariables]
              = "FLOW_DIR[" + std::to_string(Interco) + "]";
            logs.debug() << NumberOfVariables << " direct flow[" << Interco << "]. ";
            NumberOfVariables++;

            indirectVar = variableManager_.IntercoIndirectCost(Interco, triggeredHour)
              = NumberOfVariables;
            problemeAResoudre_.TypeDeVariable[NumberOfVariables] = VARIABLE_BORNEE_DES_DEUX_COTES;
            problemeAResoudre_.NomDesVariables[NumberOfVariables]
              = "FLOW_INDIR[" + std::to_string(Interco) + "]";
            logs.debug() << NumberOfVariables << " indirect flow[" << Interco << "]. ";
            NumberOfVariables++;

            linkInsideAdqPatch[algebraicFluxVar] = LinkVariable(directVar, indirectVar);
        }
    }
}
