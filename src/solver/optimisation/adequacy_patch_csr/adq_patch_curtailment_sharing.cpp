// Copyright 2007-2026, RTE (https://www.rte-france.com)
// SPDX-License-Identifier: MPL-2.0

#include "antares/solver/optimisation/adequacy_patch_csr/adq_patch_curtailment_sharing.h"

#include <cmath>
#include <spx_constantes_externes.h>

#include "antares/solver/adequacy-patch/gems-csr-adapter.h"
#include "antares/solver/optimisation/adequacy_patch_csr/count_constraints_variables.h"
#include "antares/solver/optimisation/adequacy_patch_csr/csr_quadratic_problem.h"
#include "antares/solver/simulation/adequacy_patch_runtime_data.h"

#include "solve_problem.h"

using namespace Yuni;
namespace
{
// Thin CsrProblemBuilder implementation that assigns consecutive column indices
// starting at startIdx_, storing bounds directly in PROBLEME_ANTARES_A_RESOUDRE.
class CsrColumnAllocator final : public Antares::AdequacyPatch::CsrProblemBuilder
{
public:
    CsrColumnAllocator(PROBLEME_ANTARES_A_RESOUDRE& pa, int startIdx):
        pa_(pa),
        nextIdx_(startIdx)
    {
    }

    int allocateColumn(const std::string& /*name*/, double lb, double ub) override
    {
        int col = nextIdx_++;
        pa_.Xmin[col] = lb;
        pa_.Xmax[col] = ub;
        pa_.TypeDeVariable[col] = VARIABLE_NON_BORNEE;
        return col;
    }

private:
    PROBLEME_ANTARES_A_RESOUDRE& pa_;
    int nextIdx_;
};
} // anonymous namespace

namespace Antares::Data::AdequacyPatch
{
double LmrViolationAreaHour(PROBLEME_HEBDO* problemeHebdo,
                            double totalNodeBalance,
                            double threshold,
                            int Area,
                            int hour)
{
    const double ensInit = problemeHebdo->ResultatsHoraires[Area]
                             .ValeursHorairesDeDefaillancePositive[hour];

    problemeHebdo->ResultatsHoraires[Area].ValeursHorairesLmrViolations[hour] = 0;
    // check LMR violations
    if ((ensInit > 0.0) && (totalNodeBalance < 0.0)
        && (std::fabs(totalNodeBalance) > ensInit + std::fabs(threshold)))
    {
        problemeHebdo->ResultatsHoraires[Area].ValeursHorairesLmrViolations[hour] = 1;
        return std::fabs(totalNodeBalance);
    }
    return 0.0;
}

std::tuple<double, double, double> calculateAreaFlowBalance(PROBLEME_HEBDO* problemeHebdo,
                                                            bool setNTCOutsideToInsideToZero,
                                                            int Area,
                                                            int hour)
{
    double netPositionInit = 0;
    double flowsNode1toNodeA = 0;
    double densNew;

    int Interco = problemeHebdo->IndexDebutIntercoOrigine[Area];
    while (Interco >= 0)
    {
        if (problemeHebdo->adequacyPatchRuntimeData->extremityAreaMode[Interco]
            == physicalAreaInsideAdqPatch)
        {
            netPositionInit -= problemeHebdo->ValeursDeNTC[hour].ValeurDuFlux[Interco];
        }
        else if (problemeHebdo->adequacyPatchRuntimeData->extremityAreaMode[Interco]
                 == physicalAreaOutsideAdqPatch)
        {
            flowsNode1toNodeA -= std::min(0.0,
                                          problemeHebdo->ValeursDeNTC[hour].ValeurDuFlux[Interco]);
        }
        Interco = problemeHebdo->IndexSuivantIntercoOrigine[Interco];
    }
    Interco = problemeHebdo->IndexDebutIntercoExtremite[Area];
    while (Interco >= 0)
    {
        if (problemeHebdo->adequacyPatchRuntimeData->originAreaMode[Interco]
            == physicalAreaInsideAdqPatch)
        {
            netPositionInit += problemeHebdo->ValeursDeNTC[hour].ValeurDuFlux[Interco];
        }
        else if (problemeHebdo->adequacyPatchRuntimeData->originAreaMode[Interco]
                 == physicalAreaOutsideAdqPatch)
        {
            flowsNode1toNodeA += std::max(0.0,
                                          problemeHebdo->ValeursDeNTC[hour].ValeurDuFlux[Interco]);
        }
        Interco = problemeHebdo->IndexSuivantIntercoExtremite[Interco];
    }

    double gemsContrib = 0.0;
    if (problemeHebdo->modelerData)
    {
        gemsContrib = problemeHebdo->ResultatsHoraires[Area]
                        .ValeursHorairesNetechangeModeler[hour];
        netPositionInit += gemsContrib;
    }

    double ensInit = problemeHebdo->ResultatsHoraires[Area]
                       .ValeursHorairesDeDefaillancePositive[hour];
    if (!setNTCOutsideToInsideToZero)
    {
        densNew = std::max(0.0, ensInit + netPositionInit + flowsNode1toNodeA);
        if (problemeHebdo->modelerData && (ensInit > 0.0 || gemsContrib != 0.0))
        {
            logs.info() << "[GEMS-VERIFY][H4] areaFlowBalance area=" << Area
                        << " h=" << hour << " ensInit=" << ensInit
                        << " netPosNTC=" << (netPositionInit - gemsContrib)
                        << " gemsContrib=" << gemsContrib
                        << " netPosTotal=" << netPositionInit
                        << " flowsOutside=" << flowsNode1toNodeA
                        << " densNew=" << densNew;
        }
        return std::make_tuple(netPositionInit, densNew, netPositionInit + flowsNode1toNodeA);
    }
    else
    {
        densNew = std::max(0.0, ensInit + netPositionInit);
        if (problemeHebdo->modelerData && (ensInit > 0.0 || gemsContrib != 0.0))
        {
            logs.info() << "[GEMS-VERIFY][H4] areaFlowBalance area=" << Area
                        << " h=" << hour << " ensInit=" << ensInit
                        << " netPosNTC=" << (netPositionInit - gemsContrib)
                        << " gemsContrib=" << gemsContrib
                        << " netPosTotal=" << netPositionInit
                        << " (outsideLinksZeroed) densNew=" << densNew;
        }
        return std::make_tuple(netPositionInit, densNew, netPositionInit);
    }
}

} // namespace Antares::Data::AdequacyPatch

void HourlyCSRProblem::calculateCsrParameters()
{
    using namespace Antares::Data::AdequacyPatch;
    double netPositionInit;
    int hour = triggeredHour;

    for (uint32_t Area = 0; Area < problemeHebdo_->NombreDePays; Area++)
    {
        if (problemeHebdo_->adequacyPatchRuntimeData->areaMode[Area] == physicalAreaInsideAdqPatch)
        {
            problemeHebdo_->adequacyPatchRuntimeData->addCSRTriggeredAtAreaHour(Area, hour);

            // calculate netPositionInit and the RHS of the AreaBalance constraints
            std::tie(netPositionInit, std::ignore, std::ignore) = calculateAreaFlowBalance(
              problemeHebdo_,
              adqPatchParams_.setToZeroOutsideInsideLinks,
              Area,
              hour);

            double ensInit = problemeHebdo_->ResultatsHoraires[Area]
                               .ValeursHorairesDeDefaillancePositive[hour];
            double spillageInit = problemeHebdo_->ResultatsHoraires[Area]
                                    .ValeursHorairesDeDefaillanceNegative[hour];

            rhsAreaBalanceValues[Area] = ensInit + netPositionInit - spillageInit;
            const double dens = std::max(0.0, ensInit + netPositionInit);
            logs.info() << "[ADQ-DEBUG][LMR] area='" << problemeHebdo_->NomsDesPays[Area]
                        << "' h=" << hour
                        << " ENS_init=" << ensInit
                        << " netPosInit=" << netPositionInit
                        << " DENS=" << dens
                        << " spillageInit=" << spillageInit
                        << " csrRHS=" << rhsAreaBalanceValues[Area];
            logs.info() << "[GEMS-VERIFY][H2] csrRHS area=" << Area
                        << " (" << problemeHebdo_->NomsDesPays[Area] << ")"
                        << " h=" << hour
                        << " ensInit=" << ensInit
                        << " netPosInit=" << netPositionInit
                        << " spillageInit=" << spillageInit
                        << " RHS=" << rhsAreaBalanceValues[Area];
        }
    }
}

void HourlyCSRProblem::allocateProblem()
{
    using namespace Antares::Data::AdequacyPatch;
    problemeAResoudre_.NombreDeVariables = countVariables(problemeHebdo_);
    problemeAResoudre_.NombreDeContraintes = countConstraints(problemeHebdo_);
    OPT_AllocateFromNumberOfVariableConstraints(&problemeAResoudre_);
}

void HourlyCSRProblem::buildProblemVariables()
{
    logs.debug() << "[CSR] variable list:";

    constructVariableENS();
    constructVariableSpilledEnergy();
    constructVariableFlows();

    const auto* rtd = problemeHebdo_->adequacyPatchRuntimeData.get();
    if (rtd && rtd->useGemsFbConstraints && rtd->gemsCsrAdapter)
    {
        // NombreDeVariables now equals the legacy count — use it as the start index
        // for extra GEMS columns.
        CsrColumnAllocator allocator(problemeAResoudre_,
                                     problemeAResoudre_.NombreDeVariables);
        rtd->gemsCsrAdapter->registerExtraVariables(allocator);
        // constructVariableENS() reset NombreDeVariables to 0 and rebuilt it to
        // the legacy count; advance it past the GEMS extra columns now.
        problemeAResoudre_.NombreDeVariables += rtd->gemsCsrAdapter->countExtraVariables();
    }
}

void HourlyCSRProblem::buildProblemConstraintsLHS()
{
    Antares::Solver::Optimization::CsrQuadraticProblem csrProb(problemeHebdo_,
                                                               problemeAResoudre_,
                                                               *this);
    csrProb.buildConstraintMatrix();
}

void HourlyCSRProblem::setVariableBounds()
{
    for (int var = 0; var < problemeAResoudre_.NombreDeVariables; var++)
    {
        problemeAResoudre_.AdresseOuPlacerLaValeurDesVariablesOptimisees[var] = nullptr;
    }

    logs.debug() << "[CSR] bounds";
    setBoundsOnENS();
    setBoundsOnSpilledEnergy();
    setBoundsOnFlows();
    setBoundsOnGemsFbExtraVars();
}

void HourlyCSRProblem::buildProblemConstraintsRHS()
{
    logs.debug() << "[CSR] RHS: ";
    setRHSvalueOnFlows();
    setRHSnodeBalanceValue();
    setRHSfictitiousLoadValue();
    setRHSMaxEnsLoadValue();
    setRHSbindingConstraintsValue();
    setRHSgemsFbConstraintsValue();
}

void HourlyCSRProblem::setProblemCost()
{
    logs.debug() << "[CSR] cost";
    problemeAResoudre_.CoutLineaire.assign(problemeAResoudre_.NombreDeVariables, 0.);
    problemeAResoudre_.CoutQuadratique.assign(problemeAResoudre_.NombreDeVariables, 0.);

    setQuadraticCost();
    if (adqPatchParams_.curtailmentSharing.includeHurdleCost)
    {
        setLinearCost();
    }
}

void HourlyCSRProblem::solveProblem(uint week, int year, const OptimizationOptions& options)
{
    ADQ_PATCH_CSR(options.quadraticOptimOptions,
                  problemeAResoudre_,
                  *this,
                  adqPatchParams_,
                  week,
                  year);
}

void HourlyCSRProblem::run(uint week, uint year)
{
    mcYear_ = static_cast<int>(year);
    calculateCsrParameters();
    buildProblemVariables();
    buildProblemConstraintsLHS();
    setVariableBounds();
    buildProblemConstraintsRHS();
    setProblemCost();
    solveProblem(week, year, solverOptions_);

    // Dump full CSR QP solution for validation
    {
        logs.info() << "[ADQ-DEBUG][CSR-SOL] h=" << triggeredHour
                    << " nVars=" << problemeAResoudre_.NombreDeVariables;
        for (int col = 0; col < problemeAResoudre_.NombreDeVariables; ++col)
        {
            const std::string& varName
              = (col < static_cast<int>(problemeAResoudre_.NomDesVariables.size()))
                  ? problemeAResoudre_.NomDesVariables[col]
                  : "?";
            logs.info() << "[ADQ-DEBUG][CSR-SOL]   col=" << col
                        << " name=" << varName
                        << " X=" << problemeAResoudre_.X[col];
        }
        // GEMS exchange contributions (inside + outside) from QP solution
        const auto* rtd = problemeHebdo_->adequacyPatchRuntimeData.get();
        if (rtd && rtd->useGemsFbConstraints && rtd->gemsCsrAdapter)
        {
            for (const auto& contrib : rtd->gemsCsrAdapter->areaFlowContributions())
            {
                logs.info() << "[GEMS-VERIFY][H2] QPsol h=" << triggeredHour
                            << " area='" << contrib.areaName << "' (inside)"
                            << " col=" << contrib.csrColumn
                            << " X=" << problemeAResoudre_.X[contrib.csrColumn]
                            << " coeff=" << contrib.coefficient
                            << " netContrib="
                            << (contrib.coefficient * problemeAResoudre_.X[contrib.csrColumn]);
            }
            for (const auto& contrib : rtd->gemsCsrAdapter->outsideAreaFlowContributions())
            {
                logs.info() << "[ADQ-DEBUG][CSR-SOL] QPsol h=" << triggeredHour
                            << " area='" << contrib.areaName << "' (outside)"
                            << " col=" << contrib.csrColumn
                            << " X=" << problemeAResoudre_.X[contrib.csrColumn]
                            << " coeff=" << contrib.coefficient
                            << " netContrib="
                            << (contrib.coefficient * problemeAResoudre_.X[contrib.csrColumn]);
            }
        }
    }

    updateGemsExchangeAfterCSR();
}

void HourlyCSRProblem::updateGemsExchangeAfterCSR()
{
    const auto* rtd = problemeHebdo_->adequacyPatchRuntimeData.get();
    if (!rtd || !rtd->useGemsFbConstraints || !rtd->gemsCsrAdapter)
        return;

    const auto& insideContribs = rtd->gemsCsrAdapter->areaFlowContributions();
    const auto& outsideContribs = rtd->gemsCsrAdapter->outsideAreaFlowContributions();
    if (insideContribs.empty())
        return;

    // Map area name → area index
    std::map<std::string, int> nameToIdx;
    for (uint32_t i = 0; i < problemeHebdo_->NombreDePays; ++i)
        nameToIdx[problemeHebdo_->NomsDesPays[i]] = static_cast<int>(i);

    // Log LP values for all areas before overwrite
    logs.info() << "[ADQ-DEBUG][WRITEBACK] preWrite NetechangeModeler h=" << triggeredHour;
    for (uint32_t i = 0; i < problemeHebdo_->NombreDePays; ++i)
    {
        const int mode = static_cast<int>(rtd->areaMode[i]);
        const double val = problemeHebdo_->ResultatsHoraires[i]
                             .ValeursHorairesNetechangeModeler[triggeredHour];
        logs.info() << "[ADQ-DEBUG][WRITEBACK]   area='" << problemeHebdo_->NomsDesPays[i]
                    << "' mode=" << mode << " lpVal=" << val;
    }

    // Reset all areas then write inside + outside from QP solution
    for (uint32_t i = 0; i < problemeHebdo_->NombreDePays; ++i)
        problemeHebdo_->ResultatsHoraires[i].ValeursHorairesNetechangeModeler[triggeredHour] = 0.0;

    auto writeBack = [&](const std::vector<Antares::AdequacyPatch::AreaFlowContribution>& contribs,
                         const char* tag)
    {
        for (const auto& contrib : contribs)
        {
            auto it = nameToIdx.find(contrib.areaName);
            if (it == nameToIdx.end())
                continue;
            const double qpVal = problemeAResoudre_.X[contrib.csrColumn];
            const double written = contrib.coefficient * qpVal;
            problemeHebdo_->ResultatsHoraires[it->second]
              .ValeursHorairesNetechangeModeler[triggeredHour] += written;
            logs.info() << "[ADQ-DEBUG][WRITEBACK] " << tag
                        << " area='" << contrib.areaName
                        << "' col=" << contrib.csrColumn
                        << " qpX=" << qpVal
                        << " coeff=" << contrib.coefficient
                        << " written=" << written;
        }
    };

    writeBack(insideContribs, "inside");
    writeBack(outsideContribs, "outside");

    // Log post-write values and compute energy balance check
    logs.info() << "[ADQ-DEBUG][WRITEBACK] postWrite NetechangeModeler h=" << triggeredHour;
    double balanceSum = 0.0;
    for (uint32_t i = 0; i < problemeHebdo_->NombreDePays; ++i)
    {
        const int mode = static_cast<int>(rtd->areaMode[i]);
        const double val = problemeHebdo_->ResultatsHoraires[i]
                             .ValeursHorairesNetechangeModeler[triggeredHour];
        balanceSum += val;
        logs.info() << "[ADQ-DEBUG][WRITEBACK]   area='" << problemeHebdo_->NomsDesPays[i]
                    << "' mode=" << mode << " csrVal=" << val;
    }
    logs.info() << "[ADQ-DEBUG][BALANCE-CHECK] h=" << triggeredHour
                << " sum_exchange=" << balanceSum
                << (std::fabs(balanceSum) < 1.0 ? " OK" : " VIOLATION");
}

void HourlyCSRProblem::setBoundsOnGemsFbExtraVars()
{
    const auto* rtd = problemeHebdo_->adequacyPatchRuntimeData.get();
    if (!rtd || !rtd->useGemsFbConstraints || !rtd->gemsCsrAdapter)
    {
        return;
    }
    // Step 1: default all extra GEMS columns to unbounded (they survive the
    // AdresseOuPlacerLaValeurDesVariablesOptimisees reset at the top of setVariableBounds()).
    constexpr double kInf = 1e20;
    const int legacyEnd = problemeAResoudre_.NombreDeVariables
                          - rtd->gemsCsrAdapter->countExtraVariables();
    for (int col = legacyEnd; col < problemeAResoudre_.NombreDeVariables; ++col)
    {
        problemeAResoudre_.Xmin[col] = -kInf;
        problemeAResoudre_.Xmax[col] = kInf;
        problemeAResoudre_.TypeDeVariable[col] = VARIABLE_NON_BORNEE;
        problemeAResoudre_.X[col] = 0.0;
    }

    // Step 2: apply per-hour model bounds for variables that have explicit bounds.
    const auto varBounds
      = rtd->gemsCsrAdapter->variableBoundsForHour(globalTriggeredHour, mcYear_);
    for (const auto& vb : varBounds)
    {
        problemeAResoudre_.Xmin[vb.col] = vb.lb;
        problemeAResoudre_.Xmax[vb.col] = vb.ub;
        const bool hasBothSides = (vb.lb > -kInf) && (vb.ub < kInf);
        const bool hasLbOnly = (vb.lb > -kInf) && (vb.ub >= kInf);
        if (hasBothSides)
            problemeAResoudre_.TypeDeVariable[vb.col] = VARIABLE_BORNEE_DES_DEUX_COTES;
        else if (hasLbOnly)
            problemeAResoudre_.TypeDeVariable[vb.col] = VARIABLE_BORNEE_INFERIEUREMENT;
        else
            problemeAResoudre_.TypeDeVariable[vb.col] = VARIABLE_BORNEE_SUPERIEUREMENT;
    }

    // Step 3: dump all extra-variable bounds for validation.
    for (int col = legacyEnd; col < problemeAResoudre_.NombreDeVariables; ++col)
    {
        logs.info() << "[ADQ-DEBUG][CSR-VAR] h=" << triggeredHour
                    << " col=" << col
                    << " lb=" << problemeAResoudre_.Xmin[col]
                    << " ub=" << problemeAResoudre_.Xmax[col]
                    << " type=" << problemeAResoudre_.TypeDeVariable[col]
                    << " (gems-extra)";
    }
}

void HourlyCSRProblem::setRHSgemsFbConstraintsValue()
{
    const auto* rtd = problemeHebdo_->adequacyPatchRuntimeData.get();
    if (!rtd || !rtd->useGemsFbConstraints || !rtd->gemsCsrAdapter)
    {
        return;
    }

    const auto rows = rtd->gemsCsrAdapter->rowsForHour(globalTriggeredHour, mcYear_);
    for (size_t i = 0; i < rows.size() && i < gemsFbConstraintRows_.size(); ++i)
    {
        const int csrRow = gemsFbConstraintRows_[i];
        if (csrRow >= 0)
        {
            problemeAResoudre_.SecondMembre[csrRow] = rows[i].rhs;
            logs.info() << "[GEMS-VERIFY][H3] setRHS: rowIdx=" << i
                        << " csrRow=" << csrRow
                        << " id=" << rows[i].constraintId
                        << " rhs=" << rows[i].rhs;
        }
    }
}