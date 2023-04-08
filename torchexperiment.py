

import torch, torch.nn as nn

class L1Penalty(torch.autograd.Function):
    @staticmethod
    def forward(ctx, input, l1weight = 0.1):
        ctx.save_for_backward(input)
        ctx.l1weight = l1weight
        return input

    @staticmethod
    def backward(ctx, grad_output):
        input, = ctx.saved_variables
        grad_input = input.clone().sign().mul(ctx.l1weight)
        grad_input+=grad_output
        return grad_input


