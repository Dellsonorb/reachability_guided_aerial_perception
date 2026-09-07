"""Bounded A6 experiment adapters around frozen A1–A5 components."""

from .policy import decide_policy, rm4d_top_one

__all__ = ['decide_policy', 'rm4d_top_one']
