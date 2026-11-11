function [z, confidence, available] = trn_measurement(x, env, cfg)
% Simplified TRN position fix. Dust reduces feature confidence.
confidence = exp(-cfg.trn.kDust*env.dustTau);
available = confidence >= cfg.trn.confidenceThreshold;
if available
    sigma = cfg.trn.basePositionNoise / max(confidence, 0.1);
    z = x.r(:) + sigma*randn(3,1);
else
    z = nan(3,1);
end
end
